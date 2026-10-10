#!/usr/bin/env python3
"""
Parse Dynamixel Protocol 2.0 messages from serial output hex dump.
The input file contains three columns: offset, hexadecimal values, and ASCII values.
"""

import sys
import re
import argparse
from typing import List, Dict, Optional, Tuple


class DynamixelParser:
    """Parser for Dynamixel Protocol 2.0 packets."""
    
    # Packet header for Protocol 2.0
    HEADER = [0xFF, 0xFF, 0xFD, 0x00]
    
    # Instruction types
    INSTRUCTIONS = {
        0x01: "PING",
        0x02: "READ",
        0x03: "WRITE",
        0x04: "REG_WRITE",
        0x05: "ACTION",
        0x06: "FACTORY_RESET",
        0x07: "REBOOT",
        0x08: "CLEAR",
        0x09: "CONTROL_TABLE_BACKUP",
        0x10: "STATUS",
        0x55: "STATUS",  # Status return
        0x82: "SYNC_READ",
        0x83: "SYNC_WRITE",
        0x84: "BULK_READ",
        0x92: "BULK_WRITE",
        0x93: "FAST_SYNC_READ"
    }
    
    # Packet types
    INSTRUCTION_PACKET = 0  # Sent to servos
    STATUS_PACKET = 1       # Response from servos
    
    def __init__(self):
        self.packets = []
    
    def read_hex_dump(self, filename: str) -> List[int]:
        """Read hex dump file and extract all hex bytes."""
        hex_bytes = []
        
        with open(filename, 'r') as f:
            for line in f:
                # Each line has format: "offset   hex bytes   ascii"
                # Extract the hex bytes (middle column)
                parts = line.strip().split()
                if len(parts) < 2:
                    continue
                
                # Skip the offset (first part)
                for part in parts[1:]:
                    # Stop at ASCII representation (indicated by non-hex)
                    if not re.match(r'^[0-9a-fA-F]+$', part):
                        break
                    # Convert each hex byte
                    try:
                        hex_bytes.append(int(part, 16))
                    except ValueError:
                        pass
        
        return hex_bytes
    
    def find_packets(self, data: List[int]) -> List[Dict]:
        """Find and parse all Dynamixel Protocol 2.0 packets in the data."""
        packets = []
        i = 0
        
        while i < len(data) - 4:
            # Look for packet header [0xFF, 0xFF, 0xFD, 0x00]
            if (data[i:i+4] == self.HEADER):
                packet = self.parse_packet(data, i)
                if packet:
                    packets.append(packet)
                    i += packet['total_length']
                else:
                    i += 1
            else:
                i += 1
        
        return packets
    
    def parse_packet(self, data: List[int], start: int) -> Optional[Dict]:
        """Parse a single Dynamixel Protocol 2.0 packet."""
        try:
            # Check if we have enough data for minimum packet
            if start + 10 > len(data):
                return None
            
            # Protocol 2.0 packet structure:
            # [0xFF, 0xFF, 0xFD, 0x00, ID, LEN_L, LEN_H, INST, PARAM..., CRC_L, CRC_H]
            
            idx = start + 4  # Skip header
            
            packet_id = data[idx]
            idx += 1
            
            length = data[idx] | (data[idx+1] << 8)
            idx += 2
            
            # Length includes instruction + parameters + CRC (2 bytes)
            # Total packet length = 4 (header) + 1 (ID) + 2 (length) + length
            total_length = 7 + length
            
            if start + total_length > len(data):
                return None
            
            instruction = data[idx]
            idx += 1
            
            # Parameters (length - 3: instruction(1) + CRC(2))
            param_length = length - 3
            parameters = data[idx:idx + param_length]
            idx += param_length
            
            # CRC
            crc = data[idx] | (data[idx+1] << 8)
            
            # Calculate CRC for verification
            calculated_crc = self.calculate_crc(data[start:start + total_length - 2])
            crc_valid = (crc == calculated_crc)
            
            packet = {
                'offset': start,
                'id': packet_id,
                'length': length,
                'instruction': instruction,
                'instruction_name': self.INSTRUCTIONS.get(instruction, f"UNKNOWN(0x{instruction:02X})"),
                'parameters': parameters,
                'crc': crc,
                'crc_valid': crc_valid,
                'total_length': total_length,
                'raw_bytes': data[start:start + total_length]
            }
            
            return packet
            
        except (IndexError, ValueError):
            return None
    
    def calculate_crc(self, data: List[int]) -> int:
        """Calculate CRC-16 for Dynamixel Protocol 2.0."""
        crc_table = [
            0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
            0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
            0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
            0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
            0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
            0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
            0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
            0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
            0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
            0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
            0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
            0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
            0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
            0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
            0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
            0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
            0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
            0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
            0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
            0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
            0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
            0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
            0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
            0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
            0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
            0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
            0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
            0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
            0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
            0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
            0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
            0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202
        ]
        
        crc = 0
        for byte in data:
            i = ((crc >> 8) ^ byte) & 0xFF
            crc = ((crc << 8) ^ crc_table[i]) & 0xFFFF
        
        return crc
    
    def parse_sync_write(self, packet: Dict) -> Optional[Dict]:
        """Parse SYNC_WRITE packet parameters.
        
        SYNC_WRITE format:
        [Starting Address (2 bytes), Data Length (2 bytes), 
         [ID1, Data...], [ID2, Data...], ...]
        """
        if packet['instruction'] != 0x83:  # SYNC_WRITE
            return None
        
        params = packet['parameters']
        if len(params) < 4:
            return None
        
        starting_address = params[0] | (params[1] << 8)
        data_length = params[2] | (params[3] << 8)
        
        # Parse servo data
        servo_data = {}
        idx = 4
        
        while idx < len(params):
            if idx + data_length >= len(params):
                break
            
            servo_id = params[idx]
            data = params[idx + 1:idx + 1 + data_length]
            
            servo_data[servo_id] = {
                'address': starting_address,
                'data': data,
                'data_length': data_length
            }
            
            idx += 1 + data_length
        
        return {
            'starting_address': starting_address,
            'data_length': data_length,
            'servos': servo_data
        }
    
    def parse_bulk_read(self, packet: Dict) -> Optional[Dict]:
        """Parse BULK_READ packet parameters.
        
        BULK_READ format:
        [[ID1, Starting Address (2 bytes), Data Length (2 bytes)], ...]
        """
        if packet['instruction'] != 0x84:  # BULK_READ
            return None
        
        params = packet['parameters']
        servos = {}
        idx = 0
        
        while idx + 4 < len(params):
            servo_id = params[idx]
            starting_address = params[idx + 1] | (params[idx + 2] << 8)
            data_length = params[idx + 3] | (params[idx + 4] << 8)
            
            servos[servo_id] = {
                'address': starting_address,
                'data_length': data_length
            }
            
            idx += 5
        
        return {'servos': servos}
    
    def parse_status_response(self, packet: Dict) -> Optional[Dict]:
        """Parse STATUS response packet.
        
        STATUS response format (from a single servo):
        Packet ID = servo ID responding
        Parameters = [Error, Data...]
        """
        if packet['instruction'] != 0x55:  # STATUS
            return None
        
        params = packet['parameters']
        if len(params) < 1:
            return None
        
        error = params[0]
        data = params[1:] if len(params) > 1 else []
        
        return {
            'servo_id': packet['id'],
            'error': error,
            'data': data
        }
    
    def parse_bulk_read_response(self, packets: List[Dict], start_idx: int) -> Optional[Dict]:
        """Parse BULK_READ response which consists of multiple STATUS packets.
        
        After a BULK_READ command, multiple STATUS packets follow, one per servo.
        """
        servos = {}
        idx = start_idx
        
        # Collect consecutive STATUS packets
        while idx < len(packets):
            packet = packets[idx]
            if packet['instruction'] != 0x55:  # STATUS
                break
            
            status = self.parse_status_response(packet)
            if status:
                servos[status['servo_id']] = {
                    'error': status['error'],
                    'data': status['data']
                }
            
            idx += 1
        
        return {'servos': servos, 'count': idx - start_idx} if servos else None
    
    def format_packet(self, packet: Dict) -> str:
        """Format a packet for display."""
        lines = []
        lines.append(f"\n{'='*70}")
        lines.append(f"Packet at offset 0x{packet['offset']:04X}")
        lines.append(f"  ID: 0x{packet['id']:02X} ({packet['id']})")
        lines.append(f"  Length: {packet['length']} bytes")
        lines.append(f"  Instruction: {packet['instruction_name']} (0x{packet['instruction']:02X})")
        
        if packet['parameters']:
            param_hex = ' '.join(f"{b:02X}" for b in packet['parameters'])
            lines.append(f"  Parameters ({len(packet['parameters'])} bytes): {param_hex}")
            
            # Parse SYNC_WRITE details
            if packet['instruction'] == 0x83:
                sync_data = self.parse_sync_write(packet)
                if sync_data:
                    lines.append(f"  Starting Address: 0x{sync_data['starting_address']:04X}")
                    lines.append(f"  Data Length per Servo: {sync_data['data_length']} bytes")
                    lines.append(f"  Servos: {len(sync_data['servos'])}")
                    for servo_id, data in sorted(sync_data['servos'].items()):
                        data_hex = ' '.join(f"{b:02X}" for b in data['data'])
                        lines.append(f"    ID 0x{servo_id:02X}: {data_hex}")
        
        lines.append(f"  CRC: 0x{packet['crc']:04X} {'✓' if packet['crc_valid'] else '✗ INVALID'}")
        
        raw_hex = ' '.join(f"{b:02X}" for b in packet['raw_bytes'])
        lines.append(f"  Raw: {raw_hex}")
        
        return '\n'.join(lines)


def bytes_to_value(data: List[int]) -> Optional[int]:
    """Convert byte array to integer value."""
    if len(data) >= 4:
        # 32-bit value (little-endian)
        value = (data[0] | (data[1] << 8) | 
                (data[2] << 16) | (data[3] << 24))
        # Handle signed 32-bit
        if value >= 0x80000000:
            value -= 0x100000000
        return value
    elif len(data) >= 2:
        # 16-bit value
        value = data[0] | (data[1] << 8)
        if value >= 0x8000:
            value -= 0x10000
        return value
    elif len(data) == 1:
        # 8-bit value
        return data[0]
    return None


def extract_servo_values(packets: List[Dict], parser: DynamixelParser, 
                         servo_ids: List[int]) -> List[Tuple[int, Dict[int, int]]]:
    """Extract values from SYNC_WRITE packets for specific servo IDs.
    
    Returns list of (packet_index, {servo_id: value}) tuples.
    Assumes 4-byte values (common for position/velocity).
    """
    results = []
    
    for idx, packet in enumerate(packets):
        if packet['instruction'] != 0x83:  # SYNC_WRITE
            continue
        
        sync_data = parser.parse_sync_write(packet)
        if not sync_data:
            continue
        
        values = {}
        for servo_id in servo_ids:
            if servo_id in sync_data['servos']:
                data = sync_data['servos'][servo_id]['data']
                value = bytes_to_value(data)
                if value is not None:
                    values[servo_id] = value
        
        if values:
            results.append((idx, values))
    
    return results


def extract_read_write_pairs(packets: List[Dict], parser: DynamixelParser,
                             servo_ids: List[int]) -> List[Tuple[int, Dict[int, Tuple[int, int]]]]:
    """Extract paired SYNC_WRITE (written) and SYNC_READ/BULK_READ response (read) values.
    
    Returns list of (packet_index, {servo_id: (written_value, read_value)}) tuples.
    """
    results = []
    i = 0
    
    while i < len(packets):
        packet = packets[i]
        
        # Look for SYNC_WRITE command
        if packet['instruction'] == 0x83:  # SYNC_WRITE
            sync_data = parser.parse_sync_write(packet)
            if not sync_data:
                i += 1
                continue
            
            # Extract written values
            written_values = {}
            for servo_id in servo_ids:
                if servo_id in sync_data['servos']:
                    data = sync_data['servos'][servo_id]['data']
                    value = bytes_to_value(data)
                    if value is not None:
                        written_values[servo_id] = value
            
            # Look ahead for SYNC_READ or BULK_READ command followed by STATUS responses
            j = i + 1
            read_values = {}
            
            while j < len(packets):
                next_packet = packets[j]
                
                # Found SYNC_READ or BULK_READ
                if next_packet['instruction'] in [0x82, 0x84]:  # SYNC_READ or BULK_READ
                    # Parse the following STATUS responses
                    bulk_response = parser.parse_bulk_read_response(packets, j + 1)
                    if bulk_response:
                        for servo_id in servo_ids:
                            if servo_id in bulk_response['servos']:
                                data = bulk_response['servos'][servo_id]['data']
                                value = bytes_to_value(data)
                                if value is not None:
                                    read_values[servo_id] = value
                        break
                
                # Stop if we hit another SYNC_WRITE before finding SYNC_READ/BULK_READ
                if next_packet['instruction'] == 0x83:
                    break
                
                j += 1
            
            # Create pairs for servos that have both written and read values
            pairs = {}
            for servo_id in servo_ids:
                if servo_id in written_values and servo_id in read_values:
                    pairs[servo_id] = (written_values[servo_id], read_values[servo_id])
            
            if pairs:
                results.append((i, pairs))
        
        i += 1
    
    return results


def output_gnuplot_format(results: List[Tuple[int, Dict[int, int]]], 
                         servo_ids: List[int]):
    """Output data in gnuplot-compatible format.
    
    Format: index servo_0x10 servo_0x11 servo_0x12
    """
    # Header
    header = "# Index"
    for servo_id in servo_ids:
        header += f" Servo_0x{servo_id:02X}"
    print(header)
    
    # Data rows
    for idx, values in results:
        row = f"{idx}"
        for servo_id in servo_ids:
            if servo_id in values:
                row += f" {values[servo_id]}"
            else:
                row += " ?"
        print(row)


def output_gnuplot_diff_format(results: List[Tuple[int, Dict[int, Tuple[int, int]]]], 
                               servo_ids: List[int]):
    """Output difference data in gnuplot-compatible format.
    
    For each servo, outputs: written_value read_value difference
    Format: index servo_0x10_written servo_0x10_read servo_0x10_diff ...
    """
    # Header
    header = "# Index"
    for servo_id in servo_ids:
        header += f" Servo_0x{servo_id:02X}_Written Servo_0x{servo_id:02X}_Read Servo_0x{servo_id:02X}_Diff"
    print(header)
    
    # Data rows
    for idx, pairs in results:
        row = f"{idx}"
        for servo_id in servo_ids:
            if servo_id in pairs:
                written, read = pairs[servo_id]
                diff = read - written
                row += f" {written} {read} {diff}"
            else:
                row += " ? ? ?"
        print(row)


def main():
    arg_parser = argparse.ArgumentParser(
        description='Parse Dynamixel Protocol 2.0 messages from hex dump',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='''
Examples:
  # Show all packets
  python parse_dynamixel.py file.txt
  
  # Extract servo values for gnuplot
  python parse_dynamixel.py --gnuplot file.txt
  
  # Compare written vs read values (SYNC_WRITE vs SYNC_READ/BULK_READ)
  python parse_dynamixel.py --gnuplot --diff file.txt
  
  # Specify custom servo IDs
  python parse_dynamixel.py --gnuplot --servo-ids 0x10 0x11 0x12 file.txt
  
  # Redirect to file for gnuplot
  python parse_dynamixel.py --gnuplot --diff file.txt > servo_diff.dat
  
  # Plot differences with gnuplot
  gnuplot -p -e "plot 'servo_diff.dat' using 1:4 with lines title 'Servo 0x10 Diff'"
''')
    
    arg_parser.add_argument('filename', help='Hex dump file to parse')
    arg_parser.add_argument('--gnuplot', action='store_true',
                           help='Output servo values in gnuplot format')
    arg_parser.add_argument('--diff', action='store_true',
                           help='Compare SYNC_WRITE (written) vs SYNC_READ/BULK_READ (read) values')
    arg_parser.add_argument('--servo-ids', nargs='+', 
                           default=['0x10', '0x11', '0x12'],
                           help='Servo IDs to extract (default: 0x10 0x11 0x12)')
    arg_parser.add_argument('--verbose', '-v', action='store_true',
                           help='Show detailed packet information')
    
    args = arg_parser.parse_args()
    
    # Parse servo IDs
    servo_ids = [int(sid, 0) for sid in args.servo_ids]
    
    parser = DynamixelParser()
    
    if not args.gnuplot or args.verbose:
        print(f"Reading hex dump from: {args.filename}", file=sys.stderr)
    
    data = parser.read_hex_dump(args.filename)
    
    if not args.gnuplot or args.verbose:
        print(f"Total bytes read: {len(data)}", file=sys.stderr)
        print("\nSearching for Dynamixel Protocol 2.0 packets...", file=sys.stderr)
    
    packets = parser.find_packets(data)
    
    if not args.gnuplot or args.verbose:
        print(f"Found {len(packets)} packets\n", file=sys.stderr)
    
    if args.gnuplot:
        if args.diff:
            # Extract and output difference between written and read values
            results = extract_read_write_pairs(packets, parser, servo_ids)
            if not args.verbose:
                print(f"# Found {len(results)} SYNC_WRITE/BULK_READ pairs", file=sys.stderr)
            output_gnuplot_diff_format(results, servo_ids)
        else:
            # Extract and output gnuplot format (SYNC_WRITE only)
            results = extract_servo_values(packets, parser, servo_ids)
            output_gnuplot_format(results, servo_ids)
    else:
        # Print all packets
        for i, packet in enumerate(packets, 1):
            print(f"\nPacket #{i}")
            print(parser.format_packet(packet))
        
        # Summary statistics
        print(f"\n{'='*70}")
        print("SUMMARY")
        print(f"{'='*70}")
        print(f"Total packets: {len(packets)}")
        
        # Count by instruction type
        inst_counts = {}
        for packet in packets:
            inst_name = packet['instruction_name']
            inst_counts[inst_name] = inst_counts.get(inst_name, 0) + 1
        
        print("\nPackets by instruction type:")
        for inst, count in sorted(inst_counts.items()):
            print(f"  {inst}: {count}")
        
        # Count valid/invalid CRC
        valid_crc = sum(1 for p in packets if p['crc_valid'])
        invalid_crc = len(packets) - valid_crc
        print(f"\nCRC validation:")
        print(f"  Valid: {valid_crc}")
        print(f"  Invalid: {invalid_crc}")


if __name__ == "__main__":
    main()
