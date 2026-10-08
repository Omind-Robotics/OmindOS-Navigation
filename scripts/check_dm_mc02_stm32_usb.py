"""Compile USB integration against the pinned vendor headers for Cortex-M7."""
from pathlib import Path
import subprocess
import tempfile
import shutil
from prepare_dm_mc02_usb import CDC_PATH, VENDOR_COMMIT, patch

ROOT = Path(__file__).resolve().parents[1]
HAL_RNG_COMMIT = 'b716379524ba3549e4db11804889776ffeb239d1'  # HAL v1.11.1


def main():
    with tempfile.TemporaryDirectory() as directory:
        vendor = Path(directory)/'vendor'
        vendor.mkdir()
        def git(*args):
            subprocess.run(['git', '-C', str(vendor), *args], check=True)
        git('init', '-q')
        git('remote', 'add', 'origin', 'https://github.com/dmBots/DM-MC02.git')
        git('fetch', '--depth=1', 'origin', VENDOR_COMMIT)
        git('checkout', '-q', 'FETCH_HEAD')
        project = vendor/'examples/CtrBoard-H7_ALL'
        # Vendor includes HAL v1.11.1 but omits its RNG files. Add only the
        # matching official RNG implementation, not a replacement HAL tree.
        official = Path(directory)/'official-hal'
        official.mkdir()
        for args in [
            ('init', '-q'),
            ('remote', 'add', 'origin', 'https://github.com/STMicroelectronics/stm32h7xx-hal-driver.git'),
            ('fetch', '--depth=1', 'origin', HAL_RNG_COMMIT),
            ('checkout', '-q', 'FETCH_HEAD'),
        ]:
            subprocess.run(['git', '-C', str(official), *args], check=True)
        driver = project/'Drivers/STM32H7xx_HAL_Driver'
        for folder, suffix in [('Inc', '.h'), ('Src', '.c')]:
            for module in ['stm32h7xx_hal_rng', 'stm32h7xx_hal_rng_ex']:
                shutil.copyfile(official/folder/(module+suffix),
                                driver/folder/(module+suffix))
        target = Path(directory)/'usbd_cdc_if.c'
        target.write_bytes(patch((vendor/CDC_PATH).read_bytes()))
        # Modified or repatched vendor sources must fail, before any output write.
        try:
            patch(target.read_bytes())
        except ValueError:
            pass
        else:
            raise AssertionError('repatched vendor file was accepted')
        includes = [
            ROOT/'firmware/dm_mc02', project/'Core/Inc',
            project/'Drivers/CMSIS/Include',
            project/'Drivers/CMSIS/Device/ST/STM32H7xx/Include',
            project/'Drivers/STM32H7xx_HAL_Driver/Inc',
            project/'USB_DEVICE/App', project/'USB_DEVICE/Target',
            project/'Middlewares/ST/STM32_USB_Device_Library/Core/Inc',
            project/'Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc',
        ]
        for source in (ROOT/'firmware/dm_mc02/om_stm32_usb.c', target,
                       driver/'Src/stm32h7xx_hal_rng.c',
                       driver/'Src/stm32h7xx_hal_rng_ex.c'):
            subprocess.run([
                'arm-none-eabi-gcc', '-std=c11', '-Wall', '-Wextra', '-Werror',
                '-mcpu=cortex-m7', '-mthumb', '-mfloat-abi=soft',
                '-DSTM32H723xx', '-DUSE_HAL_DRIVER', '-DHAL_RNG_MODULE_ENABLED',
                *['-I'+str(path) for path in includes],
                '-c', str(source), '-o', str(Path(directory)/(source.stem+'.o')),
            ], check=True)
        print('Pinned CDC patch and four Cortex-M7 objects passed; no linked firmware.')


if __name__ == '__main__':
    main()
