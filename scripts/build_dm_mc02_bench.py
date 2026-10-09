"""Fully link a read-only STM32H723 observer; hardware acceptance stays false."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import struct
import subprocess
from check_dm_mc02_stm32_usb import checkout, HAL_RNG_COMMIT
from prepare_dm_mc02_usb import patch, VENDOR_COMMIT, CDC_PATH

ROOT = Path(__file__).resolve().parents[1]
BENCH = ROOT/'firmware/dm_mc02/bench'
RTOS_PORT_COMMIT = '88e32327e975ddde97c390bc5b6c1f8e7d9d239e'
CPU = ['-mcpu=cortex-m7', '-mthumb', '-mfpu=fpv5-d16', '-mfloat-abi=hard']
IRQS = [f'FDCAN{b}_IT{n}_IRQHandler' for b in (1, 2, 3) for n in (0, 1)]
IRQS += ['OTG_HS_IRQHandler', 'TIM23_IRQHandler', 'SVC_Handler', 'PendSV_Handler', 'SysTick_Handler']


def output(*args):
    return subprocess.check_output(list(map(str, args)), text=True)


def routing(config):
    raw = config.read_bytes()
    data = json.loads(raw)
    if data.get('schema') != 'omindos-dm-mc02-observer-v1':
        raise ValueError('Wrong observer routing schema')
    routes = data.get('routes', [])
    if len(routes) != 12 or any(set(r) != {'bus', 'motor_id'} for r in routes):
        raise ValueError('Need exactly twelve bus/ID-only routes')
    pairs = [(r['bus'], r['motor_id']) for r in routes]
    if any(type(b) is not int or type(m) is not int or b not in range(3) or not 1 <= m <= 127 for b, m in pairs):
        raise ValueError('Invalid bus/motor ID')
    if len(set(pairs)) != 12 or any(sum(b == i for b, _ in pairs) != 4 for i in range(3)):
        raise ValueError('Need four unique IDs per bus')
    digest = hashlib.sha256(raw).digest()
    header = '#include "om_usb.h"\nstatic const uint8_t bench_digest[32]={'
    header += ','.join(str(b) for b in digest)+'};\nstatic const om_joint bench_joints[12]={\n'
    header += ',\n'.join('{.bus=%d,.motor_id=%d}' % pair for pair in pairs)+'\n};\n'
    return header, digest.hex(), data


def startup(source):
    """Use the pinned vendor's exact vector order; replace only ARMASM syntax."""
    table = source.read_text().split('__Vectors       DCD', 1)[1].split('__Vectors_End', 1)[0]
    vectors = ['_estack'] + re.findall(r'^\s+DCD\s+(\w+)', table, re.M)
    if vectors[1] != 'Reset_Handler' or len(vectors) != 179:
        raise ValueError('Pinned STM32H723 vector layout differs')
    text = '''/* Vector order from pinned ST/vendor startup; see NOTICE. */
.syntax unified
.cpu cortex-m7
.fpu fpv5-d16
.thumb
.section .isr_vector,"a",%progbits
.global g_pfnVectors
.type g_pfnVectors,%object
g_pfnVectors:
'''
    text += ''.join('  .word '+v+'\n' for v in vectors)
    text += '''.size g_pfnVectors, .-g_pfnVectors
.section .text.Reset_Handler,"ax",%progbits
.global Reset_Handler
.type Reset_Handler,%function
.thumb_func
Reset_Handler:
  bl SystemInit
  ldr r0, =_sidata
  ldr r1, =_sdata
  ldr r2, =_edata
1:
  cmp r1, r2
  bcs 2f
  ldr r3, [r0], #4
  str r3, [r1], #4
  b 1b
2:
  ldr r1, =_sbss
  ldr r2, =_ebss
  movs r3, #0
3:
  cmp r1, r2
  bcs 4f
  str r3, [r1], #4
  b 3b
4:
  bl main
  b Default_Handler
.size Reset_Handler, .-Reset_Handler
.section .text.Default_Handler,"ax",%progbits
.global Default_Handler
.type Default_Handler,%function
.thumb_func
Default_Handler:
  cpsid i
  b .
'''
    for v in sorted(set(vectors) - {'_estack', 'Reset_Handler', '0'}):
        text += f'.weak {v}\n.thumb_set {v}, Default_Handler\n'
    return text, vectors


def build(args):
    dest = args.output_dir.resolve()
    dest.mkdir(parents=True, exist_ok=False)
    vendor = args.vendor_dir.resolve()
    official = args.hal_dir.resolve()
    port = args.freertos_dir.resolve()
    dependencies = [(vendor, 'https://github.com/dmBots/DM-MC02.git', VENDOR_COMMIT),
                    (official, 'https://github.com/STMicroelectronics/stm32h7xx-hal-driver.git', HAL_RNG_COMMIT),
                    (port, 'https://github.com/FreeRTOS/FreeRTOS-Kernel.git', RTOS_PORT_COMMIT)]
    os.environ['GIT_LFS_SKIP_SMUDGE'] = '1' # CAD/media assets are not build inputs.
    for path, url, commit in dependencies:
        checkout(path, url, commit)
        if output('git', '-C', path, 'status', '--porcelain', '--untracked-files=no').strip():
            raise ValueError('Dependency working tree is not clean: '+str(path))
    project = vendor/'examples/CtrBoard-H7_ALL'
    hal = project/'Drivers/STM32H7xx_HAL_Driver'
    rtos = project/'Middlewares/Third_Party/FreeRTOS/Source'
    usb = project/'Middlewares/ST/STM32_USB_Device_Library'
    overlay = dest/'overlay'; overlay.mkdir()
    header, digest, config = routing(args.config)
    (overlay/'bench_config.h').write_text(header)
    shutil.copyfile(args.config, dest/'observer-routing.json')
    for folder, suffix in [('Inc', '.h'), ('Src', '.c')]:
        for name in ['stm32h7xx_hal_rng', 'stm32h7xx_hal_rng_ex']:
            shutil.copyfile(official/folder/(name+suffix), overlay/(name+suffix))
    conf = (project/'Core/Inc/stm32h7xx_hal_conf.h').read_text()
    enabled = {'HAL_MODULE_ENABLED', 'HAL_FDCAN_MODULE_ENABLED', 'HAL_TIM_MODULE_ENABLED',
               'HAL_PCD_MODULE_ENABLED', 'HAL_GPIO_MODULE_ENABLED', 'HAL_DMA_MODULE_ENABLED',
               'HAL_RCC_MODULE_ENABLED', 'HAL_FLASH_MODULE_ENABLED', 'HAL_PWR_MODULE_ENABLED',
               'HAL_CORTEX_MODULE_ENABLED', 'HAL_EXTI_MODULE_ENABLED'}
    conf = re.sub(r'^\s*#define (HAL_\w+_MODULE_ENABLED|HAL_MODULE_ENABLED)\b.*$',
                  lambda m: m[0] if m[1] in enabled else '/* bench excludes '+m[1]+' */', conf, flags=re.M)
    conf = conf.replace('/* #define HAL_RNG_MODULE_ENABLED   */', '#define HAL_RNG_MODULE_ENABLED')
    (overlay/'stm32h7xx_hal_conf.h').write_text(conf)
    fdcan = (project/'Core/Src/fdcan.c').read_text()
    replacements = [('FDCAN_MODE_NORMAL', 'FDCAN_MODE_BUS_MONITORING'),
                    ('Init.StdFiltersNbr = 1;', 'Init.StdFiltersNbr = 0;'),
                    ('Init.ExtFiltersNbr = 0;', 'Init.ExtFiltersNbr = 1;'),
                    ('Init.TxFifoQueueElmtsNbr = 32;', 'Init.TxFifoQueueElmtsNbr = 0;')]
    for old, new in replacements:
        if fdcan.count(old) != 3:
            raise ValueError('Pinned FDCAN init differs')
        fdcan = fdcan.replace(old, new)
    (overlay/'fdcan.c').write_text(fdcan)
    timebase = (project/'Core/Src/stm32h7xx_hal_timebase_tim.c').read_text()
    marker = 'uwTimclock = 2*HAL_RCC_GetPCLK2Freq();'
    if timebase.count(marker) != 1:
        raise ValueError('Pinned TIM23 clock calculation differs')
    timebase = timebase.replace(marker, 'uwTimclock = HAL_RCC_GetPCLK2Freq();\n'
                                       '  if(clkconfig.APB2CLKDivider != RCC_APB2_DIV1) uwTimclock *= 2;')
    (overlay/'stm32h7xx_hal_timebase_tim.c').write_text(timebase)
    main = (project/'Core/Src/main.c').read_text(errors='replace')
    clock = main.split('void SystemClock_Config(void)\n{', 1)[1].split('/* USER CODE BEGIN 4 */', 1)[0]
    clock = clock.replace('HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSI, RCC_MCODIV_1);', '')
    clock = clock.replace('HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);',
                          'if(HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY)!=HAL_OK) Error_Handler();')
    (overlay/'bench_clock.c').write_text('#include "main.h"\nvoid SystemClock_Config(void)\n{'+clock)
    (overlay/'usbd_cdc_if.c').write_bytes(patch((vendor/CDC_PATH).read_bytes()))
    usbd_conf = (project/'USB_DEVICE/Target/usbd_conf.c').read_text()
    usb_header = (project/'USB_DEVICE/Target/usbd_conf.h').read_text()
    if usb_header.count('#define USBD_LPM_ENABLED     1U') != 1:
        raise ValueError('Pinned USB LPM configuration differs')
    (overlay/'usbd_conf.h').write_text(usb_header.replace('#define USBD_LPM_ENABLED     1U',
                                                       '#define USBD_LPM_ENABLED     0U'))
    usbd_conf = usbd_conf.replace('#include "usbd_cdc.h"', '#include "usbd_cdc.h"\n#include "om_stm32_usb.h"')
    marker = 'USBD_LL_DevDisconnected((USBD_HandleTypeDef*)hpcd->pData);'
    if usbd_conf.count(marker) != 1:
        raise ValueError('Pinned USB disconnect callback differs')
    usbd_conf = usbd_conf.replace(marker, 'om_stm32_usb_disconnect();\n  '+marker)
    (overlay/'usbd_conf.c').write_text(usbd_conf)
    asm, vectors = startup(project/'MDK-ARM/startup_stm32h723xx.s')
    (overlay/'startup.S').write_text(asm)
    includes = [overlay, BENCH, ROOT/'firmware/dm_mc02', project/'Core/Inc',
                project/'Drivers/CMSIS/Include', project/'Drivers/CMSIS/Device/ST/STM32H7xx/Include',
                hal/'Inc', project/'USB_DEVICE/App', project/'USB_DEVICE/Target',
                usb/'Core/Inc', usb/'Class/CDC/Inc', rtos/'include', port/'portable/GCC/ARM_CM7/r0p1']
    sources = [ROOT/'firmware/dm_mc02'/(name+'.c') for name in
               ('om_usb', 'om_cdc', 'om_feedback', 'om_stm32_usb', 'om_stm32_can')]
    sources += [BENCH/'bench_main.c', BENCH/'bench_irq.c']
    sources += [overlay/name for name in ('fdcan.c', 'bench_clock.c', 'usbd_cdc_if.c', 'usbd_conf.c',
                                         'stm32h7xx_hal_rng.c', 'stm32h7xx_hal_rng_ex.c', 'startup.S')]
    sources += [project/'Core/Src'/name for name in ('system_stm32h7xx.c', 'stm32h7xx_hal_msp.c')]
    sources += [overlay/'stm32h7xx_hal_timebase_tim.c']
    sources += [hal/'Src'/('stm32h7xx_'+name+'.c') for name in
                ('hal', 'hal_fdcan', 'hal_cortex', 'hal_rcc', 'hal_rcc_ex', 'hal_pwr', 'hal_pwr_ex',
                 'hal_flash', 'hal_flash_ex', 'hal_gpio', 'hal_dma', 'hal_dma_ex',
                 'hal_exti', 'hal_tim', 'hal_tim_ex', 'hal_pcd', 'hal_pcd_ex', 'll_usb')]
    sources += [project/'USB_DEVICE/App'/name for name in ('usb_device.c', 'usbd_desc.c')]
    sources += [usb/'Core/Src'/name for name in ('usbd_core.c', 'usbd_ctlreq.c', 'usbd_ioreq.c')]
    sources += [usb/'Class/CDC/Src/usbd_cdc.c', rtos/'tasks.c', rtos/'list.c', port/'portable/GCC/ARM_CM7/r0p1/port.c']
    flags = [*CPU, '-std=c11', '-Os', '-g3', '-ffunction-sections', '-fdata-sections',
             '-fstack-usage', '-fno-common', '-MMD', '-Wall', '-Wextra', '-DSTM32H723xx', '-DUSE_HAL_DRIVER',
             '-DOM_BENCH_OBSERVER_ONLY', '-ffile-prefix-map='+str(ROOT)+'=omindos',
             '-ffile-prefix-map='+str(vendor)+'=vendor', '-ffile-prefix-map='+str(official)+'=hal-rng',
             '-ffile-prefix-map='+str(port)+'=freertos-port', '-ffile-prefix-map='+str(dest)+'=build',
             *['-I'+str(p) for p in includes]]
    objects = []; commands = []
    objdir = dest/'objects'; objdir.mkdir()
    for i, src in enumerate(sources):
        obj = objdir/f'{i:02d}-{src.stem}.o';objects.append(obj)
        cmd = ['arm-none-eabi-gcc', *flags]
        if src.parent in (ROOT/'firmware/dm_mc02', BENCH):
            cmd += ['-Werror']
        cmd += ['-c', str(src), '-o', str(obj)]
        commands.append(cmd); subprocess.run(cmd, check=True)
    elf = dest/'dm-mc02-observer.elf'
    cmd = ['arm-none-eabi-gcc', *CPU, '-nostartfiles', '--specs=nano.specs',
           '-T'+str(BENCH/'stm32h723.ld'), '-Wl,--gc-sections,--fatal-warnings,--print-memory-usage',
           '-Wl,-Map='+str(dest/'dm-mc02-observer.map'), *map(str, objects), '-Wl,--start-group',
           '-lc', '-lm', '-lgcc', '-Wl,--end-group', '-o', str(elf)]
    commands.append(cmd); subprocess.run(cmd, check=True)
    undefined = output('arm-none-eabi-nm', '-u', elf).strip()
    if undefined: raise AssertionError('Undefined symbols: '+undefined)
    nm = output('arm-none-eabi-nm', '-n', elf)
    (dest/'symbols.txt').write_text(nm)
    symbols = {p[2]: (int(p[0], 16), p[1]) for line in nm.splitlines() if len(p := line.split()) == 3}
    banned = [name for name in symbols if re.search(r'HAL_FDCAN_(AddMessage|EnableTx|ReqTransmit)|fdcanx_send|can_bsp|FunTest|KeyTask|IMU|LCD|om_encode_targets|^om_init$|pvPortMalloc|^malloc$|_sbrk', name)]
    if banned: raise AssertionError('Forbidden linked symbol: '+str(banned))
    for name in ['Reset_Handler', 'main', 'om_stm32_usb_init_observer', 'HAL_RNG_MspInit', *IRQS]:
        if name not in symbols or symbols[name][1] != 'T': raise AssertionError('Missing strong handler: '+name)
    for ext in ['hex', 'bin']:
        subprocess.run(['arm-none-eabi-objcopy', '-O', 'ihex' if ext == 'hex' else 'binary',
                        str(elf), str(dest/('dm-mc02-observer.'+ext))], check=True)
    binary = (dest/'dm-mc02-observer.bin').read_bytes()
    for i, name in enumerate(vectors):
        actual = struct.unpack_from('<I', binary, 4*i)[0]
        expected = 0 if name == '0' else symbols[name][0] | (0 if name == '_estack' else 1)
        if actual != expected: raise AssertionError('Vector mismatch: '+name)
    size = output('arm-none-eabi-size', '-A', elf)
    readelf = output('arm-none-eabi-readelf', '-h', '-l', '-S', '-A', elf)
    (dest/'size.txt').write_text(size); (dest/'readelf.txt').write_text(readelf)
    if symbols['_flash_end'][0]-0x08000000 != len(binary): raise AssertionError('BIN LMA mismatch')
    if not 0x24000000 <= symbols['_sbss'][0] <= symbols['_ebss'][0] <= 0x24050000:
        raise AssertionError('AXIRAM layout mismatch')
    if symbols['_estack'][0] != 0x20020000 or symbols['_stack_limit'][0] != 0x2001e000:
        raise AssertionError('MSP reservation mismatch')
    stack_usage = []
    for path in objdir.glob('*.su'):
        for line in path.read_text().splitlines():
            parts = line.split('\t'); name = parts[0].split(':')[-1]
            if len(parts) == 3 and name in symbols and symbols[name][1].upper() == 'T':
                stack_usage.append({'function': name, 'bytes': int(parts[1]), 'kind': parts[2]})
    (dest/'stack-usage.json').write_text(json.dumps(sorted(stack_usage, key=lambda r: -r['bytes']), indent=2)+'\n')
    checksums = {('dm-mc02-observer.'+ext): hashlib.sha256((dest/('dm-mc02-observer.'+ext)).read_bytes()).hexdigest()
                 for ext in ('elf', 'hex', 'bin')}
    (dest/'SHA256SUMS').write_text(''.join(f'{v}  {k}\n' for k, v in checksums.items()))
    (dest/'build-commands.json').write_text(json.dumps(commands, indent=2)+'\n')
    evidence = {'vendor_commit': VENDOR_COMMIT, 'hal_rng_commit': HAL_RNG_COMMIT,
                'freertos_gcc_port_commit': RTOS_PORT_COMMIT, 'compiler': output('arm-none-eabi-gcc', '--version').splitlines()[0],
                'cpu_flags': CPU, 'objects': len(objects), 'complete_firmware_link': True,
                'undefined_symbols': [], 'flashable_format_generated': True, 'hardware_tested': False,
                'board_flash_authorized_or_performed': False, 'bootloader_offset_verified': False,
                'flash_base': '0x08000000', 'flash_image_bytes': len(binary), 'msp_reserved_bytes': 8192,
                'axi_static_bytes': symbols['_ebss'][0]-0x24000000, 'bench_task_stack_bytes': 16384,
                'vectors_checked': len(vectors), 'strong_irq_handlers': IRQS, 'forbidden_linked_symbols': [],
                'can_mode': 'BUS_MONITORING; no data TX or ACK; TX allocation=0 from first init',
                'motion_target_codec_linked': False, 'configuration_sha256': digest,
                'usb_lpm_enabled': False, 'usb_dma_enabled': False,
                'largest_retained_function_stack_bytes': max(r['bytes'] for r in stack_usage),
                'stack_runtime_high_water_verified': False,
                'routing_verified_on_hardware': config.get('routing_verified_on_hardware', False), 'checksums': checksums}
    (dest/'validation.json').write_text(json.dumps(evidence, indent=2)+'\n')
    inputs = {*sources, BENCH/'FreeRTOSConfig.h', BENCH/'stm32h723.ld', args.config.resolve(), Path(__file__).resolve()}
    for dep in objdir.glob('*.d'):
        # GCC escapes spaces in make depfiles. Paths in this build are whitespace-free.
        if '\\ ' in dep.read_text(): raise ValueError('Use whitespace-free build paths')
        inputs.update(Path(p) for p in dep.read_text().replace('\\\n', '').split(':', 1)[1].split())
    manifest = {str(p.relative_to(ROOT)) if p.is_relative_to(ROOT) else str(p): hashlib.sha256(p.read_bytes()).hexdigest()
                for p in sorted(inputs)}
    (dest/'source-sha256.json').write_text(json.dumps(manifest, indent=2)+'\n')
    for path, _, _ in dependencies:
        if output('git', '-C', path, 'status', '--porcelain', '--untracked-files=no').strip(): raise AssertionError('Dependency modified')
    print(json.dumps(evidence, indent=2))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--vendor-dir', type=Path, default=Path('/tmp/dm-mc02-vendor'))
    parser.add_argument('--hal-dir', type=Path, default=Path('/tmp/dm-mc02-hal'))
    parser.add_argument('--freertos-dir', type=Path, default=Path('/tmp/dm-mc02-freertos'))
    parser.add_argument('--config', type=Path, default=BENCH/'observer-routing.json')
    parser.add_argument('--output-dir', type=Path, required=True, help='Must not exist')
    build(parser.parse_args())


if __name__ == '__main__':
    main()
