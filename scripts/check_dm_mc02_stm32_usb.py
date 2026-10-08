"""Compile/relocatable-link bench bindings, never claim a flashable firmware."""
import argparse
import json
from pathlib import Path
import subprocess
import tempfile
from prepare_dm_mc02_usb import CDC_PATH, VENDOR_COMMIT, patch

ROOT = Path(__file__).resolve().parents[1]
HAL_RNG_COMMIT = 'b716379524ba3549e4db11804889776ffeb239d1'


def checkout(path, url, commit):
    if not path.exists():
        path.mkdir()
        for args in [('init', '-q'), ('remote', 'add', 'origin', url),
                     ('fetch', '--depth=1', 'origin', commit), ('checkout', '-q', 'FETCH_HEAD')]:
            subprocess.run(['git', '-C', str(path), *args], check=True)
    actual = subprocess.check_output(['git', '-C', str(path), 'rev-parse', 'HEAD'], text=True).strip()
    if actual != commit:
        raise ValueError('Dependency commit mismatch: '+str(path))
    subprocess.run(['git', '-C', str(path), 'diff', '--exit-code', 'HEAD', '--'], check=True,
                   stdout=subprocess.DEVNULL)


def check(vendor, official, output):
    checkout(vendor, 'https://github.com/dmBots/DM-MC02.git', VENDOR_COMMIT)
    checkout(official, 'https://github.com/STMicroelectronics/stm32h7xx-hal-driver.git', HAL_RNG_COMMIT)
    project = vendor/'examples/CtrBoard-H7_ALL'
    driver = project/'Drivers/STM32H7xx_HAL_Driver'
    # Copy only the four matching official RNG files into a separate overlay.
    # Never alter the vendor checkout or include the entire newer HAL tree.
    overlay = output/'rng'; overlay.mkdir()
    for folder, suffix in [('Inc', '.h'), ('Src', '.c')]:
        for module in ['stm32h7xx_hal_rng', 'stm32h7xx_hal_rng_ex']:
            (overlay/(module+suffix)).write_bytes((official/folder/(module+suffix)).read_bytes())
    (overlay/'LICENSE.md').write_bytes((official/'LICENSE.md').read_bytes())
    target = output/'usbd_cdc_if.c'
    target.write_bytes(patch((vendor/CDC_PATH).read_bytes()))
    for altered in (target.read_bytes(), (vendor/CDC_PATH).read_bytes()+b'\n'):
        try:
            patch(altered)
        except ValueError:
            pass
        else:
            raise AssertionError('Changed/repatched vendor file accepted')
    includes = [ROOT/'firmware/dm_mc02', overlay, project/'Core/Inc',
                project/'Drivers/CMSIS/Include', project/'Drivers/CMSIS/Device/ST/STM32H7xx/Include',
                driver/'Inc', project/'USB_DEVICE/App', project/'USB_DEVICE/Target',
                project/'Middlewares/ST/STM32_USB_Device_Library/Core/Inc',
                project/'Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc']
    sources = [ROOT/'firmware/dm_mc02'/(module+'.c') for module in
               ('om_usb', 'om_cdc', 'om_feedback', 'om_stm32_usb', 'om_stm32_can')]
    sources += [target, overlay/'stm32h7xx_hal_rng.c', overlay/'stm32h7xx_hal_rng_ex.c']
    flags = ['-std=c11', '-Wall', '-Wextra', '-Werror', '-mcpu=cortex-m7', '-mthumb',
             '-mfloat-abi=soft', '-ffreestanding', '-DSTM32H723xx', '-DUSE_HAL_DRIVER',
             '-DHAL_RNG_MODULE_ENABLED', *['-I'+str(p) for p in includes]]
    objects = []
    for source in sources:
        obj = output/(source.stem+'.o'); objects.append(obj)
        subprocess.run(['arm-none-eabi-gcc', *flags, '-c', str(source), '-o', str(obj)], check=True)
    combined = output/'bench-bindings.relocatable.o'
    subprocess.run(['arm-none-eabi-gcc', '-mcpu=cortex-m7', '-mthumb', '-mfloat-abi=soft',
                    '-nostdlib', '-r', *map(str, objects), '-o', str(combined)], check=True)
    symbols = subprocess.check_output(['arm-none-eabi-nm', '-u', str(combined)], text=True)
    if any(x in symbols for x in ('HAL_FDCAN_AddMessage', 'fdcanx_send', 'om_stm32_', 'om_feedback_', 'om_cdc_', 'om_feed')):
        raise AssertionError('Unexpected TX reference or unresolved internal binding')
    result = {'vendor_commit': VENDOR_COMMIT, 'hal_rng_commit': HAL_RNG_COMMIT,
              'objects': len(objects), 'relocatable_link': True, 'complete_firmware_link': False,
              'hardware_tested': False, 'can_transmit_references': False,
              'compiler': subprocess.check_output(['arm-none-eabi-gcc', '--version'], text=True).splitlines()[0],
              'unresolved_external_symbols': symbols.splitlines()}
    (output/'validation.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result, indent=2))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--vendor-dir', type=Path)
    parser.add_argument('--hal-dir', type=Path)
    parser.add_argument('--output-dir', type=Path)
    args = parser.parse_args()
    with tempfile.TemporaryDirectory() as directory:
        tmp = Path(directory)
        output = args.output_dir or tmp/'objects'
        output.mkdir(parents=True, exist_ok=False)
        check(args.vendor_dir or tmp/'vendor', args.hal_dir or tmp/'official-hal', output)


if __name__ == '__main__':
    main()
