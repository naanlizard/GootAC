#!/usr/bin/env python3
import click
import subprocess
import time
import sys
import re
import serial.tools.list_ports
import os

from zeroconf import Zeroconf, ServiceBrowser

# --- GLOBAL CONFIGURATION ---
# These variables control how manage.py filters and identifies devices
DISCOVERY_PROJECT_NAME = "GootAC"         # Must match 'project' TXT record in firmware
DISCOVERY_TIMEOUT = 20                   # How long to listen for mDNS (seconds)
MDNS_SERVICE_TYPE = "_arduino._tcp.local." # Primary mDNS service to browse


# TXT Record Keys (must match src/main.cpp)
KEY_PROJECT = "project"
KEY_VERSION = "version"
KEY_UPTIME  = "uptime"
KEY_RSSI    = "rssi"
KEY_SDK     = "sdk"

class GootACDiscovery:
    def __init__(self, filter_project=DISCOVERY_PROJECT_NAME):
        self.devices = []
        self.zeroconf = Zeroconf()
        self.filter_project = filter_project

    def add_service(self, zeroconf, type, name):
        # Request full info with a 3s timeout to ensure TXT records are fetched
        info = zeroconf.get_service_info(type, name, timeout=3000)
        if info:
            ip = ".".join(map(str, info.addresses[0]))
            device_name = name.split(".")[0]

            # Map binary mDNS properties to strings
            properties = {k.decode('utf-8'): v.decode('utf-8') if v else ""
                          for k, v in info.properties.items()}

            # Filter: Is this our project?
            project_id = properties.get(KEY_PROJECT, 'Unknown')

            # Keep only if it explicitly identifies as our project OR name contains project name
            if project_id == self.filter_project or self.filter_project in device_name:
                # Avoid duplicates
                if not any(d['address'] == ip for d in self.devices):
                    self.devices.append({
                        "type": "Network",
                        "name": device_name,
                        "address": ip,
                        "version": properties.get(KEY_VERSION, "?.?.?"),
                        "uptime": properties.get(KEY_UPTIME, "N/A"),
                        "rssi": properties.get(KEY_RSSI, "N/A"),
                        "sdk": properties.get(KEY_SDK, "N/A")
                    })

    def update_service(self, zeroconf, type, name):
        pass

    def remove_service(self, zeroconf, type, name):
        pass

    def scan(self, timeout=DISCOVERY_TIMEOUT):
        """Unified scan for both Network (mDNS) and Serial (USB)"""
        # 1. Network Scan
        browser = ServiceBrowser(self.zeroconf, MDNS_SERVICE_TYPE, self)
        time.sleep(timeout)
        self.zeroconf.close()

        # 2. Serial Scan
        ports = serial.tools.list_ports.comports()
        for port in ports:
            # Common ESP8266 USB-to-Serial descriptors
            description = port.description or ""
            is_usb = any(desc.lower() in description.lower() for desc in ["CP2102", "CH340", "USB Serial", "USB-Serial"])
            is_mac_usb = "usbserial" in port.device.lower() or "wchusbserial" in port.device.lower()

            if is_usb or is_mac_usb:
                self.devices.append({
                    "type": "USB/Serial",
                    "name": port.description,
                    "address": port.device,
                    "version": "Local",
                    "uptime": "-",
                    "rssi": "-",
                    "sdk": "-"
                })

    def get_devices(self):
        return sorted(self.devices, key=lambda d: (d['type'], d['name']))

def get_config_details():
    """Extract FW_VERSION and DEVICE_NAME from src/config.h"""
    details = {"version": "0.0.0", "device_name": "GootAC"}
    try:
        if os.path.exists("src/config.h"):
            with open("src/config.h", "r") as f:
                content = f.read()
                v_match = re.search(r'#define\s+FW_VERSION\s+"([^"]+)"', content)
                d_match = re.search(r'#define\s+DEVICE_NAME\s+"([^"]+)"', content)
                if v_match: details["version"] = v_match.group(1)
                if d_match: details["device_name"] = d_match.group(1)
    except Exception:
        pass
    return details

def update_config_device_name(new_name):
    """Overwrite DEVICE_NAME in src/config.h"""
    try:
        if os.path.exists("src/config.h"):
            with open("src/config.h", "r") as f:
                content = f.read()

            content = re.sub(r'(#define\s+DEVICE_NAME\s+")[^"]+(")', rf'\1{new_name}\2', content)

            with open("src/config.h", "w") as f:
                f.write(content)
            return True
    except Exception as e:
        click.echo(f"[!] Error updating config.h: {e}")
    return False

def hostname_to_device_name(hostname):
    """Extract DEVICE_NAME from a hostname like 'GootAC-Bedroom' -> 'Bedroom'"""
    prefix = "GootAC-"
    if hostname.startswith(prefix):
        return hostname[len(prefix):]
    return hostname

def parse_version(v_str):
    """Simple semver parser for comparison"""
    try:
        return [int(x) for x in re.findall(r'\d+', v_str)]
    except:
        return [0, 0, 0]

def run_pio_command(command, port=None):
    """Executes a PlatformIO command. Returns True on success."""
    # Always specify the environment for consistency
    full_cmd = ["pio", "run", "-s", "-e", "d1_mini", "-t"] + command
    if port:
        full_cmd += ["--upload-port", port]

    click.echo(f"[*] Executing: {' '.join(full_cmd)}")
    try:
        subprocess.run(full_cmd, check=True)
        return True
    except subprocess.CalledProcessError:
        click.echo("[!] Command failed!")
        return False

def get_device_mac(port):
    """Execute esptool to read MAC address from device"""
    cmd = ["pio", "pkg", "exec", "-p", "tool-esptoolpy", "--", "esptool.py", "--port", port, "read_mac"]
    click.echo(f"[*] Reading hardware MAC address from {port}...")
    try:
        result = subprocess.run(cmd, capture_output=True, text=True, check=True)
        # Find MAC: b4:e6:2d:2a:29:9f
        match = re.search(r'MAC:\s+([a-fA-F0-9:]+)', result.stdout)
        if match:
            mac_str = match.group(1).replace(':', '').upper()
            return mac_str[-6:] # Last 6 hex digits
    except Exception as e:
        click.echo(f"[!] Warning: Could not read MAC address: {e}")
    return "XXXXXX"

def display_devices_table(devices, title="Discovered Devices", target_version=None):
    if not devices:
        click.echo("[!] No devices found.")
        return False

    config = get_config_details()
    target_version = target_version or config["version"]

    click.echo(f"\n{title}:")
    # Table Header
    header = f"{'#':<3} {'Type':<12} {'Name/Host':<20} {'Address':<15} {'Ver':<8} {'Uptime':<8} {'RSSI':<6} {'SDK':<10}"
    click.echo(header)
    click.echo("-" * len(header))

    for i, dev in enumerate(devices, 1):
        ver_str = dev['version']
        name = dev['name']
        is_old = False

        if dev['type'] == "USB/Serial":
            ver_str = "-"
        else:
            if target_version and dev['version'] != "?.?.?":
                if parse_version(dev['version']) < parse_version(target_version):
                    is_old = True
                    ver_str = click.style(dev['version'], fg='red', bold=True)

        # Adjust padding for ANSI codes if necessary, but click.echo treats stylized strings specially
        # Actually, for table alignment, it's easier to use click.echo with style segments
        line_num = f"{i:<3}"
        dev_type = f"{dev['type']:<12}"
        dev_name = f"{name[:20]:<20}"
        addr = f"{dev['address']:<15}"
        uptime = f"{dev['uptime']:<8}"
        rssi = f"{dev['rssi']:<6}"
        sdk = f"{dev['sdk']:<10}"

        click.echo(f"{line_num} {dev_type} {dev_name} {addr} ", nl=False)
        click.echo(f"{ver_str:<8}" if not is_old else ver_str + " " * (8 - len(dev['version'])), nl=False)
        click.echo(f" {uptime} {rssi} {sdk}")

    click.echo("")
    return True

def discover_all_devices(timeout=DISCOVERY_TIMEOUT):
    """Unified discovery logic used by different commands"""
    discovery = GootACDiscovery()
    discovery.scan(timeout)
    return discovery.get_devices()

@click.group()
def cli():
    """GootAC - Mitsubishi HomeKit Controller Management Utility"""
    pass

@cli.command()
@click.option('--timeout', default=DISCOVERY_TIMEOUT, help='Discovery timeout in seconds')
def list(timeout):
    """List discovered GootAC devices with full diagnostics"""
    click.echo(f"[*] Searching for {DISCOVERY_PROJECT_NAME} devices for {timeout}s...")
    devices = discover_all_devices(timeout)
    display_devices_table(devices, title="Network & Serial Devices")

@cli.command()
@click.option('--all', is_flag=True, help='Update all devices found')
@click.option('--ip', help='Update a specific device by IP address')
@click.option('--name', help='Override device name for this update (best used with --ip)')
@click.option('--timeout', default=DISCOVERY_TIMEOUT, help='Discovery timeout in seconds')
def update(all, ip, name, timeout):
    """Update firmware on selected or all devices (retains flash/pairings)"""
    original_config = get_config_details()
    target_version = original_config["version"]
    original_device_name = original_config["device_name"]

    selected_devices = []

    # 1. Fast-path: Explicit Target
    if ip and name:
        click.echo(f"[*] IP and Name provided. Skipping mDNS discovery.")
        selected_devices = [{"name": name, "address": ip, "version": "Unknown", "type": "Forced"}]
    else:
        # 2. Normal Path: Discovery
        click.echo(f"[*] Searching for {DISCOVERY_PROJECT_NAME} devices for {timeout}s...")
        devices = discover_all_devices(timeout)

        if not devices:
            click.echo("[!] No network devices found.")
            if ip:
                click.echo("[*] Tip: Provide --name with --ip to force an update without discovery.")
            return

        if ip:
            selected_devices = [d for d in devices if d['address'] == ip]
            if not selected_devices:
                click.echo(f"[!] Could not find device with IP {ip} in mDNS discovery.")
                click.echo("[*] Tip: Provide --name with --ip to force an update without discovery.")
                return
            click.echo(f"[*] Targeting specific device: {selected_devices[0]['name']} ({ip})")
        else:
            # Show full table for interactive selection
            display_devices_table(devices, title="Current Network State", target_version=target_version)

            selected_devices = []
            outdated = [d for d in devices if d['version'] != "?.?.?" and parse_version(d['version']) < parse_version(target_version)]

            if all:
                selected_devices = devices
            elif outdated:
                if click.confirm(f"[?] Detected {len(outdated)} outdated device(s). Update them to v{target_version} now?", default=True):
                    selected_devices = outdated

            if not selected_devices:
                selection = click.prompt("Enter numbers (e.g. 1,2,3), 'all', or 'q' to quit", default="all")
                if selection.lower() == 'q':
                    click.echo("[*] Aborted.")
                    return
                elif selection.lower() == 'all':
                    selected_devices = devices
                else:
                    try:
                        indices = [int(idx.strip()) - 1 for idx in selection.replace(',', ' ').split()]
                        selected_devices = [devices[idx] for idx in indices]
                    except (ValueError, IndexError):
                        click.echo("[!] Invalid selection.")
                        return

    for dev in selected_devices:
        # Derive DEVICE_NAME from mDNS hostname (e.g., "GootAC-Bedroom" -> "Bedroom")
        target_device_name = name if (name and len(selected_devices) == 1) else hostname_to_device_name(dev['name'])
        needs_swap = (target_device_name != original_device_name)

        try:
            if needs_swap:
                click.echo(f"[*] Temporarily setting DEVICE_NAME to: {target_device_name}")
                if not update_config_device_name(target_device_name):
                    click.echo("[!] Failed to update config.h. Aborting.")
                    return

            click.echo(f"\n[>] Updating {dev['name']} @ {dev['address']}...")

            # Clean build as requested
            click.echo("[*] Performing clean build...")
            run_pio_command(["clean"])

            # Upload firmware
            if run_pio_command(["upload"], port=dev['address']):
                click.echo(f"[+] Successfully updated {dev['name']}")
            else:
                click.echo(f"[!] Failed to update {dev['name']}")

        finally:
            if needs_swap:
                click.echo(f"[*] Restoring DEVICE_NAME to: {original_device_name}")
                update_config_device_name(original_device_name)


@cli.command()
@click.option('--port', help='Specific USB/Serial port to use')
@click.option('--no-erase', is_flag=True, help='Skip erasing flash before install')
def install(port, no_erase):
    """Initial setup for NEW ESP8266 devices (WIPES flash/pairings)"""
    # 1. Config Check
    if not os.path.exists("src/config.h"):
        click.echo("[!] Error: 'src/config.h' not found!")
        click.echo("[!] Copy 'src/config.h.example' to 'src/config.h' and edit it first.")
        return

    with open("src/config.h", "r") as f:
        config_content = f.read()
        if 'YOUR_SSID' in config_content or 'YOUR_PASSWORD' in config_content:
            click.echo(click.style("\n[!] Warning: Defaults detected in src/config.h!", fg='yellow', bold=True))
            if not click.confirm("[?] Are you sure you've entered your real WiFi credentials?", default=False):
                click.echo("[!] Aborted. Update your config and try again.")
                return

    # 2. Selection
    target_port = port
    target_name = "Specified Port"
    if not target_port:
        # Search for Serial devices (short timeout since it's local)
        devices = [d for d in discover_all_devices(timeout=2) if d['type'] == "USB/Serial"]

        if not devices:
            click.echo("[!] No Serial devices detected. Check connection or specify port with --port.")
            return

        click.echo("\nSelect device for INITIAL INSTALL (USB/Serial):")
        for i, dev in enumerate(devices, 1):
            click.echo(f"[{i}] {dev['name']} ({dev['address']})")

        idx = click.prompt("\nEnter number (or 'q' to quit)", default="1")
        if idx.lower() == 'q':
            return

        try:
            target_device = devices[int(idx)-1]
            target_port = target_device['address']
            target_name = target_device['name']
        except (ValueError, IndexError):
            click.echo("[!] Invalid selection.")
            return

    # 3. Naming Prompt
    mac_bits = get_device_mac(target_port)
    default_device_name = mac_bits
    click.echo(f"\n[*] Device MAC suffix: {mac_bits}")
    target_device_name = click.prompt("Enter device name (e.g., Bedroom, Suite)", default=default_device_name)

    # 4. Confirmation
    derived_hostname = f"GootAC-{target_device_name}"
    derived_accessory = f"{target_device_name} AC"
    click.echo(click.style("\n" + "="*40, fg='red'))
    click.echo(click.style("DANGER: FULL DEVICE WIPE", fg='red', bold=True))
    click.echo(f"Port:      {target_port}")
    click.echo(f"Name:      {target_device_name}")
    click.echo(f"Hostname:  {derived_hostname}")
    click.echo(f"Accessory: {derived_accessory}")
    click.echo(click.style("="*40, fg='red'))
    click.echo("[!] This will ERASE all HomeKit pairings and WiFi configuration.")
    click.echo("[!] This is required for new devices or to perform a full factory reset.")

    if not click.confirm("\n[?] Are you sure you want to proceed with a full wipe and install?", default=False):
        click.echo("[!] Aborted.")
        return

    # 5. Execution
    original_config = get_config_details()
    original_device_name = original_config["device_name"]
    should_restore = False

    try:
        if target_device_name != original_device_name:
            click.echo(f"[*] Temporarily setting DEVICE_NAME to: {target_device_name}")
            if update_config_device_name(target_device_name):
                should_restore = True

        click.echo(f"\n[*] Starting install process on {target_port}...")

        # Erase flash is recommended for first install to clear old WiFi/HomeKit info
        if not no_erase:
            click.echo("[*] Erasing old flash data...")
            if not run_pio_command(["erase"], port=target_port):
                click.echo("[!] Erase failed! Trying to flash anyway...")

        click.echo("[*] Compiling and uploading GootAC firmware...")
        if run_pio_command(["upload"], port=target_port):
            click.echo(click.style("\n[+] Success! GootAC was installed over Serial.", fg='green', bold=True))
            click.echo("[*] The device has been factory reset and is ready for HomeKit pairing.")
            click.echo("[*] Tip: Use './manage.py list' to find its new IP address once it joins WiFi.")
        else:
            click.echo(click.style("\n[!] Install failed.", fg='red', bold=True))

    finally:
        if should_restore:
            click.echo(f"[*] Restoring DEVICE_NAME to: {original_device_name}")
            update_config_device_name(original_device_name)

@cli.command()
@click.option('--port', help='Specific serial port to debug')
def debug_usb(port):
    """View detailed hardware info for USB/Serial devices"""
    ports = serial.tools.list_ports.comports()
    target_ports = [p for p in ports if p.device == port] if port else ports

    if not target_ports:
        click.echo("[!] No devices found.")
        return

    for p in target_ports:
        click.echo(f"\n[ Port: {p.device} ]")
        click.echo(f"  Description:  {p.description}")
        click.echo(f"  Manufacturer: {getattr(p, 'manufacturer', 'N/A')}")
        click.echo(f"  HWID:         {p.hwid}")
        click.echo(f"  VID:PID:      {p.vid:04X}:{p.pid:04X}" if p.vid else "  VID:PID:      N/A")
        click.echo(f"  Serial Number:{p.serial_number or 'N/A'}")
        click.echo(f"  Location:     {p.location or 'N/A'}")
        click.echo("-" * 40)

@cli.command()
@click.option('--port', help='Specific serial port to monitor')
def monitor(port):
    """Open the Serial Monitor for debugging"""
    target_port = port
    if not target_port:
        devices = [d for d in discover_all_devices(timeout=2) if d['type'] == "USB/Serial"]

        if not devices:
            click.echo("[!] No Serial devices found to monitor.")
            return

        if len(devices) == 1:
            target_port = devices[0]['address']
        else:
            for i, dev in enumerate(devices, 1):
                click.echo(f"[{i}] {dev['name']} ({dev['address']})")
            idx = click.prompt("\nEnter number to monitor", type=int)
            target_port = devices[idx-1]['address']

    run_pio_command(["monitor"], port=target_port)



if __name__ == "__main__":
    cli()
