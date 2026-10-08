"""Patch only the pinned vendor CDC callbacks; never fabricate calibration."""
import argparse
import hashlib
from pathlib import Path

VENDOR_COMMIT = 'd65a3688981405dc205f628f1a026bde8cf8303e'
CDC_BLOB = '1d0c9b82ce8a9e206125ecfcf12abb030f89e1fd'
CDC_PATH = 'examples/CtrBoard-H7_ALL/USB_DEVICE/App/usbd_cdc_if.c'


def patch(source: bytes) -> bytes:
    digest = hashlib.sha1(b'blob ' + str(len(source)).encode() + b'\0' + source).hexdigest()
    if digest != CDC_BLOB:
        raise ValueError('Vendor CDC file differs from pinned source; no write performed')
    text = source.decode('utf-8')
    changes = [
        ('#include "usbd_cdc_if.h"', '#include "usbd_cdc_if.h"\n#include "om_stm32_usb.h"'),
        ('/* USER CODE BEGIN 9 */', '/* USER CODE BEGIN 9 */\n  om_stm32_usb_disconnect();'),
        ('/* USER CODE BEGIN 11 */', '/* USER CODE BEGIN 11 */\n  om_stm32_usb_receive(Buf, *Len);'),
        ('/* USER CODE BEGIN 14 */', '/* USER CODE BEGIN 14 */\n  om_stm32_usb_complete(Buf, *Len);'),
    ]
    for old, new in changes:
        if text.count(old) != 1:
            raise ValueError('Vendor callback marker mismatch; no write performed')
        text = text.replace(old, new)
    return text.encode('utf-8')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('vendor_checkout', type=Path)
    parser.add_argument('output', type=Path, help='new CDC file; existing output is rejected')
    args = parser.parse_args()
    content = patch((args.vendor_checkout / CDC_PATH).read_bytes())
    with args.output.open('xb') as output:
        output.write(content)


if __name__ == '__main__':
    main()
