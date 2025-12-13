#!/usr/bin/env python3
"""
Code Generator for FRC Robot Electrical Contract

Reads electrical_contract_config.json (PRIMARY DATA SOURCE) and generates
Contract2025.java that is fully compatible with existing common library code.

Usage:
    python generate_contract.py
    
This will read electrical_contract_config.json and generate/overwrite:
    src/main/java/competition/electrical_contract/Contract2025.java

Benefits:
- JSON is the single source of truth
- Java code is auto-generated (no manual sync needed)
- No changes to common library classes required
- Your secondary program reads the same JSON file
- Version control tracks JSON changes (not verbose Java)
"""

import json
import sys
from pathlib import Path
from typing import Dict, List, Any


class ContractGenerator:
    """Generates Java electrical contract from JSON configuration"""

    def __init__(self, config_path: str):
        with open(config_path, 'r') as f:
            self.config = json.load(f)
        
        self.indent = "    "
        self.lines: List[str] = []

    def _auto_generate_getter_method(self, name: str, prefix: str = "get") -> str:
        """Auto-generate getter method name from device name"""
        return f"{prefix}{name}"
    
    def _auto_generate_ready_method(self, name: str) -> str:
        """Auto-generate ready method name from device name"""
        return f"is{name}Ready"
    
    def _convert_can_bus_name(self, can_bus: str) -> str:
        """Convert JSON CAN bus name to Java constant name"""
        # Map JSON config values to actual Java constant names
        can_bus_mapping = {
            "RIO": "RIO",
            "Canivore": "DefaultCanivore",
            "DefaultCanivore": "DefaultCanivore"
        }
        return can_bus_mapping.get(can_bus, can_bus)

    def generate(self) -> str:
        """Generate complete Java contract file"""
        self.lines = []
        
        self._generate_header()
        self._generate_imports()
        self._generate_class_declaration()
        self._generate_constructor()
        self._generate_drive_ready()
        self._generate_motors()
        self._generate_encoders()
        self._generate_digital_io()
        self._generate_swerve_modules()
        self._generate_imu()
        self._generate_cameras()
        self._generate_robot_dimensions()
        self._generate_additional_methods()
        self._generate_class_close()
        
        return '\n'.join(self.lines)

    def _add_line(self, line: str = "", indent_level: int = 0):
        """Add a line with proper indentation"""
        if line:
            self.lines.append(self.indent * indent_level + line)
        else:
            self.lines.append("")

    def _generate_header(self):
        """Generate file header with warning"""
        self._add_line("/*")
        self._add_line(" * AUTO-GENERATED FILE - DO NOT EDIT MANUALLY")
        self._add_line(" * ")
        self._add_line(f" * Generated from: electrical_contract_config.json")
        self._add_line(" * Generator: generate_contract.py")
        self._add_line(" * ")
        self._add_line(" * To modify this file:")
        self._add_line(" * 1. Edit electrical_contract_config.json")
        self._add_line(" * 2. Run: python generate_contract.py")
        self._add_line(" * 3. Commit both JSON and generated Java file")
        self._add_line(" */")
        self._add_line()

    def _generate_imports(self):
        """Generate import statements"""
        package = self.config.get('packageName', 'competition.electrical_contract')
        self._add_line(f"package {package};")
        self._add_line()
        
        imports = [
            "competition.subsystems.pose.PoseSubsystem",
            "edu.wpi.first.math.geometry.Rotation3d",
            "edu.wpi.first.math.geometry.Transform3d",
            "edu.wpi.first.math.geometry.Translation3d",
            "edu.wpi.first.units.measure.Distance",
            "xbot.common.controls.sensors.XGyro",
            "xbot.common.injection.electrical_contract.CANBusId",
            "xbot.common.injection.electrical_contract.CANMotorControllerInfo",
            "xbot.common.injection.electrical_contract.CANMotorControllerOutputConfig",
            "xbot.common.injection.electrical_contract.CANMotorControllerOutputConfig.InversionType",
            "xbot.common.injection.electrical_contract.CameraInfo",
            "xbot.common.injection.electrical_contract.DeviceInfo",
            "xbot.common.injection.electrical_contract.IMUInfo",
            "xbot.common.injection.electrical_contract.MotorControllerType",
            "xbot.common.injection.swerve.SwerveInstance",
            "xbot.common.math.XYPair",
            "xbot.common.subsystems.vision.CameraCapabilities",
            "",
            "javax.inject.Inject",
            "java.util.EnumSet",
            "",
            "static edu.wpi.first.units.Units.Amps",
            "static edu.wpi.first.units.Units.Inches"
        ]
        
        for imp in imports:
            if imp:
                self._add_line(f"import {imp};")
            else:
                self._add_line()

    def _generate_class_declaration(self):
        """Generate class declaration"""
        self._add_line()
        class_name = self.config.get('className', 'Contract2025')
        extends = self.config.get('extendsClass', 'ElectricalContract')
        
        self._add_line(f"public class {class_name} extends {extends} {{")
        self._add_line()

    def _generate_constructor(self):
        """Generate constructor"""
        self._add_line("@Inject", 1)
        self._add_line(f"public {self.config.get('className', 'Contract2025')}() {{}}", 1)
        self._add_line()

    def _generate_drive_ready(self):
        """Generate isDriveReady method"""
        self._add_line("@Override", 1)
        self._add_line("public boolean isDriveReady() {", 1)
        self._add_line("return true;", 2)
        self._add_line("}", 1)
        self._add_line()

    def _generate_motors(self):
        """Generate motor definitions and getters"""
        motors = self.config.get('motors', [])
        
        for motor in motors:
            self._add_line(f"// ========== {motor['name']} ==========", 1)
            self._add_line()
            
            # Auto-generate method names if not specified
            ready_method = motor.get('readyMethod', self._auto_generate_ready_method(motor['name']))
            getter_method = motor.get('getterMethod', self._auto_generate_getter_method(motor['name']))
            
            # Ready method
            self._add_line(f"public boolean {ready_method}() {{ return true; }}", 1)
            self._add_line()
            
            # Getter method
            self._add_line(f"public CANMotorControllerInfo {getter_method}() {{", 1)
            self._add_line(f"return new CANMotorControllerInfo(", 2)
            self._add_line(f'"{motor["name"]}",', 3)
            self._add_line(f'MotorControllerType.{motor["type"]},', 3)
            self._add_line(f'CANBusId.{self._convert_can_bus_name(motor["canBus"])},', 3)
            self._add_line(f'{motor["canId"]},', 3)
            
            # Output config
            config_parts = []
            if motor.get('inverted'):
                config_parts.append('withInversionType(InversionType.Inverted)')
            else:
                config_parts.append('withInversionType(InversionType.Normal)')
            
            if 'statorCurrentLimit' in motor:
                config_parts.append(f'withStatorCurrentLimit(Amps.of({motor["statorCurrentLimit"]}))')
            
            if 'supplyCurrentLimit' in motor:
                config_parts.append(f'withSupplyCurrentLimit(Amps.of({motor["supplyCurrentLimit"]}))')
            
            neutral_mode = motor.get('neutralMode', 'Coast')
            config_parts.append(f'withNeutralMode(CANMotorControllerOutputConfig.NeutralMode.{neutral_mode})')
            
            self._add_line(f'new CANMotorControllerOutputConfig()', 3)
            for part in config_parts:
                self._add_line(f'.{part}', 4)
            
            self._add_line(");", 2)
            self._add_line("}", 1)
            self._add_line()

    def _generate_encoders(self):
        """Generate encoder definitions"""
        encoders = self.config.get('encoders', [])
        
        for encoder in encoders:
            self._add_line(f"// ========== {encoder['name']} ==========", 1)
            self._add_line()
            
            # Auto-generate method names if not specified
            ready_method = encoder.get('readyMethod', self._auto_generate_ready_method(encoder['name']))
            getter_method = encoder.get('getterMethod', self._auto_generate_getter_method(encoder['name']))
            
            # Ready method
            self._add_line(f"public boolean {ready_method}() {{ return false; }}", 1)
            self._add_line()
            
            # Getter method
            self._add_line(f"public DeviceInfo {getter_method}() {{", 1)
            self._add_line(f"return new DeviceInfo(", 2)
            self._add_line(f'"{encoder["name"]}",', 3)
            self._add_line(f'CANBusId.{self._convert_can_bus_name(encoder["canBus"])},', 3)
            self._add_line(f'{encoder["canId"]},', 3)
            self._add_line(f'{str(encoder.get("inverted", False)).lower()}', 3)
            self._add_line(");", 2)
            self._add_line("}", 1)
            self._add_line()

    def _generate_digital_io(self):
        """Generate digital I/O definitions"""
        dio_devices = self.config.get('digitalIO', [])
        
        for dio in dio_devices:
            self._add_line(f"// ========== {dio['name']} ==========", 1)
            self._add_line()
            
            # Auto-generate method names if not specified
            ready_method = dio.get('readyMethod', self._auto_generate_ready_method(dio['name']))
            getter_method = dio.get('getterMethod', self._auto_generate_getter_method(dio['name']))
            
            # Ready method
            self._add_line(f"public boolean {ready_method}() {{ return true; }}", 1)
            self._add_line()
            
            # Getter method
            self._add_line("@Override", 1)
            self._add_line(f"public DeviceInfo {getter_method}() {{", 1)
            
            args = [
                f'"{dio["name"]}"',
                str(dio["channel"]),
                str(dio.get("inverted", False)).lower()
            ]
            
            if 'simulationScalingValue' in dio:
                args.append(str(dio['simulationScalingValue']))
            
            self._add_line(f"return new DeviceInfo({', '.join(args)});", 2)
            self._add_line("}", 1)
            self._add_line()

    def _generate_swerve_modules(self):
        """Generate swerve module methods"""
        modules = self.config.get('swerveModules', [])
        
        self._add_line("// ========== SWERVE DRIVE ==========", 1)
        self._add_line()
        
        # Helper methods for naming
        self._add_line("protected String getDriveControllerName(SwerveInstance swerveInstance) {", 1)
        self._add_line('return "DriveSubsystem/" + swerveInstance.label() + "/Drive";', 2)
        self._add_line("}", 1)
        self._add_line()
        
        self._add_line("protected String getSteeringControllerName(SwerveInstance swerveInstance) {", 1)
        self._add_line('return "DriveSubsystem/" + swerveInstance.label() + "/Steering";', 2)
        self._add_line("}", 1)
        self._add_line()
        
        self._add_line("protected String getSteeringEncoderControllerName(SwerveInstance swerveInstance) {", 1)
        self._add_line('return "DriveSubsystem/" + swerveInstance.label() + "/SteeringEncoder";', 2)
        self._add_line("}", 1)
        self._add_line()
        
        # getDriveMotor
        self._add_line("@Override", 1)
        self._add_line("public CANMotorControllerInfo getDriveMotor(SwerveInstance swerveInstance) {", 1)
        self._add_line("return switch (swerveInstance.label()) {", 2)
        
        for module in modules:
            motor = module['driveMotor']
            self._add_line(f'case "{module["name"]}" -> new CANMotorControllerInfo(', 3)
            self._add_line(f'getDriveControllerName(swerveInstance),', 4)
            self._add_line(f'MotorControllerType.{motor["type"]},', 4)
            self._add_line(f'CANBusId.{self._convert_can_bus_name(motor["canBus"])},', 4)
            self._add_line(f'{motor["canId"]},', 4)
            
            config_parts = []
            if motor.get('inverted'):
                config_parts.append('withInversionType(InversionType.Inverted)')
            if 'statorCurrentLimit' in motor:
                config_parts.append(f'withStatorCurrentLimit(Amps.of({motor["statorCurrentLimit"]}))')
            neutral = motor.get('neutralMode', 'Brake')
            config_parts.append(f'withNeutralMode(CANMotorControllerOutputConfig.NeutralMode.{neutral})')
            
            self._add_line('new CANMotorControllerOutputConfig()', 4)
            for part in config_parts:
                self._add_line(f'.{part}', 5)
            self._add_line(");", 4)
        
        self._add_line("default -> null;", 3)
        self._add_line("};", 2)
        self._add_line("}", 1)
        self._add_line()
        
        # getSteeringMotor
        self._add_line("@Override", 1)
        self._add_line("public CANMotorControllerInfo getSteeringMotor(SwerveInstance swerveInstance) {", 1)
        self._add_line("return switch (swerveInstance.label()) {", 2)
        
        for module in modules:
            motor = module['steeringMotor']
            self._add_line(f'case "{module["name"]}" -> new CANMotorControllerInfo(', 3)
            self._add_line(f'getSteeringControllerName(swerveInstance),', 4)
            self._add_line(f'MotorControllerType.{motor["type"]},', 4)
            self._add_line(f'CANBusId.{self._convert_can_bus_name(motor["canBus"])},', 4)
            self._add_line(f'{motor["canId"]},', 4)
            
            config_parts = []
            if motor.get('inverted'):
                config_parts.append('withInversionType(InversionType.Inverted)')
            if 'statorCurrentLimit' in motor:
                config_parts.append(f'withStatorCurrentLimit(Amps.of({motor["statorCurrentLimit"]}))')
            neutral = motor.get('neutralMode', 'Brake')
            config_parts.append(f'withNeutralMode(CANMotorControllerOutputConfig.NeutralMode.{neutral})')
            
            self._add_line('new CANMotorControllerOutputConfig()', 4)
            for part in config_parts:
                self._add_line(f'.{part}', 5)
            self._add_line(");", 4)
        
        self._add_line("default -> null;", 3)
        self._add_line("};", 2)
        self._add_line("}", 1)
        self._add_line()
        
        # getSteeringEncoder
        self._add_line("@Override", 1)
        self._add_line("public DeviceInfo getSteeringEncoder(SwerveInstance swerveInstance) {", 1)
        self._add_line("return switch (swerveInstance.label()) {", 2)
        
        for module in modules:
            encoder = module['encoder']
            self._add_line(f'case "{module["name"]}" -> new DeviceInfo(', 3)
            self._add_line(f'getSteeringEncoderControllerName(swerveInstance),', 4)
            self._add_line(f'CANBusId.{self._convert_can_bus_name(encoder["canBus"])},', 4)
            self._add_line(f'{encoder["canId"]},', 4)
            self._add_line(f'{str(encoder.get("inverted", False)).lower()}', 4)
            self._add_line(");", 4)
        
        self._add_line("default -> null;", 3)
        self._add_line("};", 2)
        self._add_line("}", 1)
        self._add_line()
        
        # getSwerveModuleOffsetsInInches
        self._add_line("@Override", 1)
        self._add_line("public XYPair getSwerveModuleOffsetsInInches(SwerveInstance swerveInstance) {", 1)
        self._add_line("return switch (swerveInstance.label()) {", 2)
        
        for module in modules:
            offset = module['offsetInches']
            self._add_line(f'case "{module["name"]}" -> new XYPair({offset["x"]}, {offset["y"]});', 3)
        
        self._add_line("default -> new XYPair(0, 0);", 3)
        self._add_line("};", 2)
        self._add_line("}", 1)
        self._add_line()

    def _generate_imu(self):
        """Generate IMU definitions"""
        imus = self.config.get('imu', [])
        
        self._add_line("// ========== IMU ==========", 1)
        self._add_line()
        
        for imu in imus:
            self._add_line("@Override", 1)
            self._add_line(f"public IMUInfo {imu['getterMethod']}() {{", 1)
            
            args = [
                f'"{imu["name"]}"',
                f'XGyro.ImuType.{imu["imuType"]}'
            ]
            
            if 'interfaceType' in imu:
                args.append(f'XGyro.InterfaceType.{imu["interfaceType"]}')
            else:
                args.append('null')
            
            if 'canBus' in imu:
                args.append(f'CANBusId.{self._convert_can_bus_name(imu["canBus"])}')
            else:
                args.append('null')
            
            args.append(str(imu.get('deviceId', 0)))
            
            self._add_line(f"return new IMUInfo({', '.join(args)});", 2)
            self._add_line("}", 1)
            self._add_line()

    def _generate_cameras(self):
        """Generate camera definitions"""
        cameras = self.config.get('cameras', [])
        
        if not cameras:
            return
        
        self._add_line("// ========== CAMERAS ==========", 1)
        self._add_line()
        
        self._add_line("public CameraInfo[] getCameraInfo() {", 1)
        self._add_line("return new CameraInfo[] {", 2)
        
        for i, cam in enumerate(cameras):
            trans = cam['transformFromRobotCenter']['translation']
            rot = cam['transformFromRobotCenter']['rotation']
            
            # Convert units
            if trans['unit'] == 'inches':
                x = trans['x'] / 39.3701  # inches to meters
                y = trans['y'] / 39.3701
                z = trans['z'] / 39.3701
            else:
                x, y, z = trans['x'], trans['y'], trans['z']
            
            if rot['unit'] == 'degrees':
                roll = f"Math.toRadians({rot['roll']})"
                pitch = f"Math.toRadians({rot['pitch']})"
                yaw = f"Math.toRadians({rot['yaw']})" if rot['yaw'] != 180 else "Math.PI"
            else:
                roll, pitch, yaw = rot['roll'], rot['pitch'], rot['yaw']
            
            self._add_line(f'new CameraInfo(', 3)
            self._add_line(f'"{cam["cameraName"]}",', 4)
            self._add_line(f'"{cam["networkTablesName"]}",', 4)
            self._add_line(f'new Transform3d(', 4)
            self._add_line(f'new Translation3d({x}, {y}, {z}),', 5)
            self._add_line(f'new Rotation3d(0, {pitch}, {yaw})', 5)
            self._add_line('),', 4)
            
            caps = ', '.join([f'CameraCapabilities.{c}' for c in cam['capabilities']])
            
            if not cam.get('enabled', True):
                self._add_line(f'EnumSet.of({caps}),', 4)
                self._add_line('false', 4)
            else:
                self._add_line(f'EnumSet.of({caps})', 4)
            
            comma = ',' if i < len(cameras) - 1 else ''
            self._add_line(f'){comma}', 3)
        
        self._add_line("};", 2)
        self._add_line("}", 1)
        self._add_line()

    def _generate_robot_dimensions(self):
        """Generate robot dimension methods"""
        dims = self.config.get('robotDimensions', {})
        
        self._add_line("// ========== ROBOT DIMENSIONS ==========", 1)
        self._add_line()
        
        if 'steeringGearRatio' in dims:
            self._add_line("@Override", 1)
            self._add_line("public double getSteeringGearRatio() {", 1)
            self._add_line(f"return {dims['steeringGearRatio']};", 2)
            self._add_line("}", 1)
            self._add_line()
        
        if 'driveGearRatio' in dims:
            self._add_line("@Override", 1)
            self._add_line("public double getDriveGearRatio() {", 1)
            self._add_line(f"return {dims['driveGearRatio']};", 2)
            self._add_line("}", 1)
            self._add_line()
        
        if 'bumperToCenter' in dims:
            bumper = dims['bumperToCenter']
            self._add_line("@Override", 1)
            self._add_line("public Distance getDistanceFromCenterToOuterBumperX() {", 1)
            self._add_line(f"return Inches.of({bumper['x']});", 2)
            self._add_line("}", 1)
            self._add_line()

    def _generate_additional_methods(self):
        """Generate additional required methods from base class"""
        self._add_line("// ========== ADDITIONAL METHODS ==========", 1)
        self._add_line()
        
        self._add_line("@Override", 1)
        self._add_line("public boolean isHumanLoadRampReady() {", 1)
        self._add_line("return false;", 2)
        self._add_line("}", 1)
        self._add_line()
        
        self._add_line("@Override", 1)
        self._add_line("public boolean areCanCodersReady() {", 1)
        self._add_line("return true;", 2)
        self._add_line("}", 1)
        self._add_line()
        
        self._add_line("@Override", 1)
        self._add_line("public boolean isDeadWheelOdometryReady() {", 1)
        self._add_line("return true;", 2)
        self._add_line("}", 1)

    def _generate_class_close(self):
        """Close the class"""
        self._add_line("}", 0)


def main():
    config_file = 'electrical_contract_config.json'
    output_file = '../src/main/java/competition/electrical_contract/Contract2025.java'
    
    print("=" * 60)
    print("Electrical Contract Code Generator")
    print("=" * 60)
    
    # Check if config exists
    if not Path(config_file).exists():
        print(f"❌ Error: {config_file} not found!")
        print(f"   Please create the configuration file first.")
        sys.exit(1)
    
    print(f"📖 Reading configuration from: {config_file}")
    
    try:
        generator = ContractGenerator(config_file)
        java_code = generator.generate()
        
        # Create output directory if needed
        output_path = Path(output_file)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        
        # Write generated code
        with open(output_file, 'w') as f:
            f.write(java_code)
        
        print(f"✅ Generated: {output_file}")
        print(f"   Lines: {len(java_code.splitlines())}")
        print()
        print("Next steps:")
        print("  1. Review the generated Contract2025.java")
        print("  2. Build your robot code to verify compilation")
        print("  3. Commit both JSON and Java files to version control")
        print()
        print("To make changes:")
        print(f"  1. Edit {config_file}")
        print("  2. Run this script again")
        
    except Exception as e:
        print(f"❌ Error generating code: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == '__main__':
    main()
