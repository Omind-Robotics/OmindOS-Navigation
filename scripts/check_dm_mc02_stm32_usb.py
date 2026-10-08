"""Compile USB integration against the pinned vendor headers for Cortex-M7."""
from pathlib import Path
import subprocess
import tempfile
from prepare_dm_mc02_usb import CDC_PATH, VENDOR_COMMIT, patch

ROOT = Path(__file__).resolve().parents[1]


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
        for source in (ROOT/'firmware/dm_mc02/om_stm32_usb.c', target):
            subprocess.run([
                'arm-none-eabi-gcc', '-std=c11', '-Wall', '-Wextra', '-Werror',
                '-mcpu=cortex-m7', '-mthumb', '-mfloat-abi=soft',
                '-DSTM32H723xx', '-DUSE_HAL_DRIVER',
                *['-I'+str(path) for path in includes],
                '-c', str(source), '-o', str(Path(directory)/(source.stem+'.o')),
            ], check=True)
        print('Pinned CDC patch and two Cortex-M7 objects passed; no linked firmware.')


if __name__ == '__main__':
    main()
