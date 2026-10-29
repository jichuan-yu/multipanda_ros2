#!/usr/bin/env python3
"""List connected SpaceMouse devices with their HID paths."""

from easyhid import Enumeration


def list_spacemouse_devices():
    """Enumerate all connected 3Dconnexion SpaceMouse devices."""
    try:
        hid = Enumeration()
    except Exception as e:
        print(f"Failed to initialize HID enumeration: {e}")
        return

    all_hids = hid.find()
    if not all_hids:
        print("No HID devices found.")
        return

    # 3Dconnexion vendor IDs
    vendor_ids = {0x256F, 0x046D}

    # Deduplicate by path (one physical device exposes multiple HID interfaces)
    seen_paths = set()
    spacemice = []
    for dev in all_hids:
        if dev.vendor_id in vendor_ids and dev.path not in seen_paths:
            seen_paths.add(dev.path)
            spacemice.append(dev)

    if not spacemice:
        print("No 3Dconnexion SpaceMouse devices found.")
        print("\nAll HID devices:")
        for dev in all_hids:
            print(f"  path={dev.path}  vendor={dev.vendor_id:04X}  product={dev.product_id:04X}  "
                  f"'{dev.manufacturer_string}' '{dev.product_string}'")
        return

    print(f"Found {len(spacemice)} SpaceMouse device(s) ({len(all_hids)} HID interfaces total):\n")
    for i, dev in enumerate(spacemice):
        print(f"  [{i}] path={dev.path}")
        print(f"       vendor_id=0x{dev.vendor_id:04X}  product_id=0x{dev.product_id:04X}")
        print(f"       manufacturer='{dev.manufacturer_string}'  product='{dev.product_string}'")
        print()

    if len(spacemice) == 2:
        print("Tip: To tell which device is left/right, unplug one and run this script again.")
        print("     The remaining path belongs to the still-connected device.")
        print()
        print("     Then in your config:")
        print(f"       left:  spacemouse_path='{spacemice[0].path}'")
        print(f"       right: spacemouse_path='{spacemice[1].path}'")


if __name__ == '__main__':
    list_spacemouse_devices()
