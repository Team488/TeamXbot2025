#!/usr/bin/env python3
"""
Example Python script demonstrating how to read and process
the electrical contract JSON file for external tools.

This could be used for:
- Generating wiring diagrams
- Creating bill of materials (BOM)
- Validating CAN ID uniqueness
- Documenting robot configuration
- Circuit simulation
"""

import json
from typing import Dict, List, Any
from collections import defaultdict


class ElectricalContractAnalyzer:
    """Analyzes electrical contract JSON data"""

    def __init__(self, json_filepath: str):
        with open(json_filepath, 'r') as f:
            self.data = json.load(f)

    def validate_can_ids(self) -> List[Dict[str, Any]]:
        """Check for duplicate CAN IDs on the same bus"""
        conflicts = []
        bus_ids = defaultdict(list)

        # Check motors
        for motor in self.data.get('motors', []):
            key = f"{motor['canBus']}:{motor['canId']}"
            bus_ids[key].append({
                'type': 'Motor',
                'name': motor['name'],
                'canBus': motor['canBus'],
                'canId': motor['canId']
            })

        # Check swerve modules
        for module in self.data.get('swerveModules', []):
            for motor_type in ['driveMotor', 'steeringMotor']:
                if motor_type in module:
                    motor = module[motor_type]
                    key = f"{motor['canBus']}:{motor['canId']}"
                    bus_ids[key].append({
                        'type': 'Motor',
                        'name': motor['name'],
                        'canBus': motor['canBus'],
                        'canId': motor['canId']
                    })
            if 'encoder' in module:
                encoder = module['encoder']
                key = f"{encoder['canBus']}:{encoder['canId']}"
                bus_ids[key].append({
                    'type': 'Encoder',
                    'name': encoder['name'],
                    'canBus': encoder['canBus'],
                    'canId': encoder['canId']
                })

        # Check encoders
        for encoder in self.data.get('encoders', []):
            key = f"{encoder['canBus']}:{encoder['canId']}"
            bus_ids[key].append({
                'type': 'Encoder',
                'name': encoder['name'],
                'canBus': encoder['canBus'],
                'canId': encoder['canId']
            })

        # Check IMUs
        for imu in self.data.get('imu', []):
            if imu.get('canBus'):
                key = f"{imu['canBus']}:{imu['deviceId']}"
                bus_ids[key].append({
                    'type': 'IMU',
                    'name': imu['name'],
                    'canBus': imu['canBus'],
                    'canId': imu['deviceId']
                })

        # Find conflicts
        for key, devices in bus_ids.items():
            if len(devices) > 1:
                conflicts.append({
                    'bus_and_id': key,
                    'devices': devices
                })

        return conflicts

    def generate_bom(self) -> Dict[str, int]:
        """Generate bill of materials"""
        bom = defaultdict(int)

        # Count motors by type
        for motor in self.data.get('motors', []):
            bom[f"Motor - {motor['type']}"] += 1

        for module in self.data.get('swerveModules', []):
            if 'driveMotor' in module:
                bom[f"Motor - {module['driveMotor']['type']}"] += 1
            if 'steeringMotor' in module:
                bom[f"Motor - {module['steeringMotor']['type']}"] += 1
            if 'encoder' in module:
                bom[f"Encoder - CANCoder"] += 1

        # Count encoders
        for encoder in self.data.get('encoders', []):
            bom[f"Encoder - CANCoder"] += 1

        # Count DIO
        bom['RoboRIO DIO'] = len(self.data.get('digitalIO', []))

        # Count IMUs
        for imu in self.data.get('imu', []):
            bom[f"IMU - {imu['imuType']}"] += 1

        # Count cameras
        for cam in self.data.get('cameras', []):
            bom[f"Camera"] += 1

        return dict(bom)

    def list_can_devices(self) -> List[Dict[str, Any]]:
        """List all CAN devices with their IDs and bus (combining conflicts)"""
        # First collect all devices grouped by bus:id
        bus_id_devices = defaultdict(list)

        # Motors
        for motor in self.data.get('motors', []):
            key = f"{motor['canBus']}:{motor['canId']}"
            bus_id_devices[key].append({
                'canBus': motor['canBus'],
                'canId': motor['canId'],
                'type': 'Motor',
                'name': motor['name']
            })

        # Swerve modules
        for module in self.data.get('swerveModules', []):
            for motor_type in ['driveMotor', 'steeringMotor']:
                if motor_type in module:
                    motor = module[motor_type]
                    key = f"{motor['canBus']}:{motor['canId']}"
                    bus_id_devices[key].append({
                        'canBus': motor['canBus'],
                        'canId': motor['canId'],
                        'type': 'Motor',
                        'name': motor['name']
                    })
            if 'encoder' in module:
                encoder = module['encoder']
                key = f"{encoder['canBus']}:{encoder['canId']}"
                bus_id_devices[key].append({
                    'canBus': encoder['canBus'],
                    'canId': encoder['canId'],
                    'type': 'Encoder',
                    'name': encoder['name']
                })

        # Encoders
        for encoder in self.data.get('encoders', []):
            key = f"{encoder['canBus']}:{encoder['canId']}"
            bus_id_devices[key].append({
                'canBus': encoder['canBus'],
                'canId': encoder['canId'],
                'type': 'Encoder',
                'name': encoder['name']
            })

        # IMUs with CAN
        for imu in self.data.get('imu', []):
            if imu.get('canBus'):
                key = f"{imu['canBus']}:{imu['deviceId']}"
                bus_id_devices[key].append({
                    'canBus': imu['canBus'],
                    'canId': imu['deviceId'],
                    'type': 'IMU',
                    'name': imu['name']
                })

        # PDH
        pdh = self.data.get('pdh', {})
        if pdh.get('canId') is not None:
            key = f"RIO:{pdh['canId']}"
            bus_id_devices[key].append({
                'canBus': 'RIO',
                'canId': pdh['canId'],
                'type': 'PDH',
                'name': 'Power Distribution Hub'
            })

        # Now convert to list, combining conflicts
        can_devices = []
        for key, devices in bus_id_devices.items():
            if len(devices) == 1:
                # No conflict
                can_devices.append(devices[0])
            else:
                # Conflict - combine into one entry
                device_names = [d['name'] for d in devices]
                can_devices.append({
                    'canBus': devices[0]['canBus'],
                    'canId': devices[0]['canId'],
                    'type': devices[0]['type'],
                    'name': f"[!] CONFLICT: {', '.join(device_names)}"
                })

        # Sort by bus and then ID
        can_devices.sort(key=lambda x: (x['canBus'], x['canId']))
        return can_devices

    def validate_pdh_channels(self) -> List[Dict[str, Any]]:
        """Check for PDH channels used by multiple devices"""
        conflicts = []
        channel_devices = defaultdict(list)
        
        # Helper function to extract PDH channel number from PowerFrom field
        def extract_pdh_channel(power_from: str) -> int:
            if power_from.startswith("PDH-"):
                return int(power_from.split("-")[1])
            return None
        
        # Check motors
        for motor in self.data.get('motors', []):
            power_from = motor.get('PowerFrom', '')
            channel = extract_pdh_channel(power_from)
            if channel is not None:
                channel_devices[channel].append({
                    'type': 'Motor',
                    'name': motor['name']
                })
        
        # Check swerve module motors
        for module in self.data.get('swerveModules', []):
            for motor_type in ['driveMotor', 'steeringMotor']:
                if motor_type in module:
                    motor = module[motor_type]
                    power_from = motor.get('PowerFrom', '')
                    channel = extract_pdh_channel(power_from)
                    if channel is not None:
                        channel_devices[channel].append({
                            'type': 'Motor',
                            'name': motor['name']
                        })
        
        # Check RIO
        rio = self.data.get('rio', {})
        rio_power = rio.get('powerFrom', '')
        channel = extract_pdh_channel(rio_power)
        if channel is not None:
            channel_devices[channel].append({
                'type': 'RoboRIO',
                'name': 'RoboRIO'
            })
        
        # Check Radio (skip if it's a range like "PDH Port 20-23")
        radio = self.data.get('radio', {})
        radio_power = radio.get('powerFrom', '')
        if "Port" not in radio_power and "-" not in radio_power.replace("PDH-", "", 1):
            channel = extract_pdh_channel(radio_power)
            if channel is not None:
                channel_devices[channel].append({
                    'type': 'Radio',
                    'name': 'Radio'
                })
        
        # Check other devices that might have powerFrom
        for device_key in ['vrm', 'pneumaticHub', 'canivore', 'pigeon2', 'coprocessor']:
            devices = self.data.get(device_key, [])
            if isinstance(devices, list):
                for device in devices:
                    power_from = device.get('powerFrom', '')
                    channel = extract_pdh_channel(power_from)
                    if channel is not None:
                        device_name = device.get('name', device_key)
                        channel_devices[channel].append({
                            'type': device_key,
                            'name': device_name
                        })
        
        # Find conflicts (channels used more than once)
        for channel, devices in channel_devices.items():
            if len(devices) > 1:
                conflicts.append({
                    'channel': channel,
                    'devices': devices
                })
        
        return conflicts

    def list_pdh_channels(self) -> List[Dict[str, Any]]:
        """List all PDH channels and their usage (derived from PowerFrom fields)"""
        # Initialize all channels as empty lists
        channel_devices = {i: [] for i in range(24)}
        
        # Helper function to extract PDH channel number from PowerFrom field
        def extract_pdh_channel(power_from: str) -> int:
            if power_from.startswith("PDH-"):
                return int(power_from.split("-")[1])
            return None
        
        # Check motors
        for motor in self.data.get('motors', []):
            power_from = motor.get('PowerFrom', '')
            channel = extract_pdh_channel(power_from)
            if channel is not None:
                channel_devices[channel].append(motor['name'])
        
        # Check swerve module motors
        for module in self.data.get('swerveModules', []):
            for motor_type in ['driveMotor', 'steeringMotor']:
                if motor_type in module:
                    motor = module[motor_type]
                    power_from = motor.get('PowerFrom', '')
                    channel = extract_pdh_channel(power_from)
                    if channel is not None:
                        channel_devices[channel].append(motor['name'])
        
        # Check RIO
        rio = self.data.get('rio', {})
        rio_power = rio.get('powerFrom', '')
        channel = extract_pdh_channel(rio_power)
        if channel is not None:
            channel_devices[channel].append("RoboRIO")
        
        # Check Radio (might be a range like "PDH Port 20-23")
        radio = self.data.get('radio', {})
        radio_power = radio.get('powerFrom', '')
        if "20-23" in radio_power or "Port 20-23" in radio_power:
            for ch in [20, 21, 22, 23]:
                channel_devices[ch].append("Radio")
        else:
            channel = extract_pdh_channel(radio_power)
            if channel is not None:
                channel_devices[channel].append("Radio")
        
        # Check other devices that might have powerFrom
        for device_key in ['vrm', 'pneumaticHub', 'canivore', 'pigeon2', 'coprocessor']:
            devices = self.data.get(device_key, [])
            if isinstance(devices, list):
                for device in devices:
                    power_from = device.get('powerFrom', '')
                    channel = extract_pdh_channel(power_from)
                    if channel is not None:
                        device_name = device.get('name', device_key)
                        channel_devices[channel].append(device_name)
        
        # Convert to list format with usage string
        channels = []
        for i in range(24):
            devices = channel_devices[i]
            if len(devices) == 0:
                usage = "Not Used"
            elif len(devices) == 1:
                usage = devices[0]
            else:
                # Multiple devices - show conflict with [!]
                usage = f"[!] CONFLICT: {', '.join(devices)}"
            
            channels.append({
                'channel': i,
                'usage': usage
            })
        
        return channels

    def list_devices_by_subsystem(self) -> Dict[str, List[str]]:
        """Organize devices by subsystem"""
        subsystems = defaultdict(list)

        # Motors
        for motor in self.data.get('motors', []):
            subsystems[motor.get('subsystem', 'Unknown')].append(
                f"Motor: {motor['name']} (CAN {motor['canId']})"
            )

        # Digital IO
        for dio in self.data.get('digitalIO', []):
            subsystems[dio.get('subsystem', 'Unknown')].append(
                f"DIO: {dio['name']} (Ch {dio['channel']})"
            )

        # Encoders
        for encoder in self.data.get('encoders', []):
            subsystems[encoder.get('subsystem', 'Unknown')].append(
                f"Encoder: {encoder['name']} (CAN {encoder['canId']})"
            )

        return dict(subsystems)

    def calculate_power_requirements(self) -> Dict[str, Any]:
        """Estimate power requirements"""
        results = {
            'motor_count': 0,
            'total_stator_limit': 0,
            'devices_by_power_source': defaultdict(list)
        }

        # Check motors
        for motor in self.data.get('motors', []):
            results['motor_count'] += 1
            if motor.get('currentLimit', {}).get('stator'):
                results['total_stator_limit'] += motor['currentLimit']['stator']
            
            power_source = motor.get('powerSource', 'Unknown')
            results['devices_by_power_source'][power_source].append(motor['name'])

        # Check swerve modules
        for module in self.data.get('swerveModules', []):
            for motor_type in ['driveMotor', 'steeringMotor']:
                if motor_type in module:
                    motor = module[motor_type]
                    results['motor_count'] += 1
                    if motor.get('currentLimit', {}).get('stator'):
                        results['total_stator_limit'] += motor['currentLimit']['stator']
                    
                    power_source = motor.get('powerSource', 'Unknown')
                    results['devices_by_power_source'][power_source].append(motor['name'])

        results['devices_by_power_source'] = dict(results['devices_by_power_source'])
        return results

    def generate_markdown_report(self) -> str:
        """Generate a markdown documentation report"""
        md = f"# {self.data.get('robotName', 'Robot')} Electrical Configuration\n\n"
        md += f"**Version:** {self.data.get('contractVersion', 'Unknown')}\n"
        md += f"**Last Updated:** {self.data.get('lastUpdated', 'Unknown')}\n\n"

        # Motors section
        md += "## Motors\n\n"
        md += "| Name | Type | CAN Bus | CAN ID | Subsystem | Current Limit (A) | Purpose |\n"
        md += "|------|------|---------|--------|-----------|-------------------|----------|\n"
        
        for motor in self.data.get('motors', []):
            stator = motor.get('currentLimit', {}).get('stator', 'N/A')
            md += f"| {motor['name']} | {motor['type']} | {motor['canBus']} | "
            md += f"{motor['canId']} | {motor['subsystem']} | {stator} | {motor['purpose']} |\n"

        # Digital IO section
        md += "\n## Digital I/O\n\n"
        md += "| Name | Channel | Inverted | Subsystem | Purpose |\n"
        md += "|------|---------|----------|-----------|----------|\n"
        
        for dio in self.data.get('digitalIO', []):
            inverted = "Yes" if dio.get('inverted') else "No"
            md += f"| {dio['name']} | {dio['channel']} | {inverted} | "
            md += f"{dio['subsystem']} | {dio['purpose']} |\n"

        # Cameras section
        md += "\n## Cameras\n\n"
        for cam in self.data.get('cameras', []):
            md += f"### {cam['cameraName']}\n"
            md += f"- **NetworkTables:** {cam['networkTablesName']}\n"
            md += f"- **Capabilities:** {', '.join(cam['capabilities'])}\n"
            trans = cam['transformFromRobotCenter']['translation']
            md += f"- **Position:** ({trans['x']:.2f}, {trans['y']:.2f}, {trans['z']:.2f}) inches\n\n"

        return md


def main():
    # Example usage
    analyzer = ElectricalContractAnalyzer('electrical_contract_config.json')

    print("=" * 60)
    print("ELECTRICAL CONTRACT ANALYSIS")
    print("=" * 60)

    # Check for CAN ID conflicts
    print("\n1. CAN ID Validation")
    print("-" * 60)
    conflicts = analyzer.validate_can_ids()
    if conflicts:
        print("WARNING: CAN ID conflicts found!")
        for conflict in conflicts:
            print(f"  Bus:ID {conflict['bus_and_id']} used by:")
            for device in conflict['devices']:
                print(f"    - {device['type']}: {device['name']}")
    else:
        print("[OK] No CAN ID conflicts detected")
    
    # Check for PDH channel conflicts
    print("\n2. PDH Channel Validation")
    print("-" * 60)
    pdh_conflicts = analyzer.validate_pdh_channels()
    if pdh_conflicts:
        print("WARNING: PDH channel conflicts found!")
        for conflict in pdh_conflicts:
            print(f"  Channel {conflict['channel']} used by {len(conflict['devices'])} devices:")
            for device in conflict['devices']:
                print(f"    - {device['type']}: {device['name']}")
    else:
        print("[OK] No PDH channel conflicts detected")

    # Generate BOM
    print("\n3. Bill of Materials")
    print("-" * 60)
    bom = analyzer.generate_bom()
    for item, count in sorted(bom.items()):
        print(f"  {item}: {count}")

    # Power analysis
    print("\n4. Power Requirements")
    print("-" * 60)
    power = analyzer.calculate_power_requirements()
    print(f"  Total motors: {power['motor_count']}")
    print(f"  Combined stator current limit: {power['total_stator_limit']} A")
    print(f"\n  Devices by power source:")
    for source, devices in power['devices_by_power_source'].items():
        print(f"    {source}: {len(devices)} devices")

    # CAN devices table
    print("\n5. CAN Device Table")
    print("-" * 60)
    can_devices = analyzer.list_can_devices()
    print(f"  {'CAN Bus':<15} {'CAN ID':<8} {'Type':<12} {'Device Name'}")
    print(f"  {'-'*15} {'-'*8} {'-'*12} {'-'*40}")
    for device in can_devices:
        print(f"  {device['canBus']:<15} {device['canId']:<8} {device['type']:<12} {device['name']}")

    # PDH channels table
    print("\n6. PDH Channel Usage")
    print("-" * 60)
    pdh_channels = analyzer.list_pdh_channels()
    print(f"  {'Channel':<10} {'Usage'}")
    print(f"  {'-'*10} {'-'*40}")
    for ch in pdh_channels:
        print(f"  {ch['channel']:<10} {ch['usage']}")

    # Subsystems
    print("\n7. Devices by Subsystem")
    print("-" * 60)
    subsystems = analyzer.list_devices_by_subsystem()
    for subsystem, devices in sorted(subsystems.items()):
        print(f"\n  {subsystem}:")
        for device in devices:
            print(f"    - {device}")

    # Generate markdown report
    print("\n8. Generating markdown report...")
    md_report = analyzer.generate_markdown_report()
    with open('electrical_contract_report.md', 'w') as f:
        f.write(md_report)
    print("   [OK] Report saved to electrical_contract_report.md")

    # Summary
    print("\n" + "=" * 60)
    total_errors = len(conflicts) + len(pdh_conflicts)
    print("SUMMARY")
    print("=" * 60)
    if total_errors == 0:
        print("No errors found - configuration is valid!")
    else:
        print(f"Total errors found: {total_errors}")
        print(f"  - CAN ID conflicts: {len(conflicts)}")
        print(f"  - PDH channel conflicts: {len(pdh_conflicts)}")
        print("\nPlease fix the errors above before deploying.")
    print("=" * 60)


if __name__ == '__main__':
    main()
