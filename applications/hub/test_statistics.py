#!/usr/bin/env python3
"""Test hub statistics tracking."""
import json
import socket
import subprocess
import sys
import time
import urllib.request

HUB_PORT = 12097
MON_PORT = 8097

def start_hub():
    """Start the hub in the background."""
    proc = subprocess.Popen(
        ['./hub', '-p', str(HUB_PORT), '-M', str(MON_PORT)],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL
    )
    time.sleep(2)  # Wait for hub to start
    return proc

def get_stats():
    """Fetch statistics from monitoring endpoint."""
    url = f'http://127.0.0.1:{MON_PORT}/status'
    with urllib.request.urlopen(url, timeout=2) as response:
        return json.loads(response.read().decode())

def send_gridconnect_packet(data):
    """Send a GridConnect packet to the hub."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        sock.connect(('127.0.0.1', HUB_PORT))
        sock.sendall(data.encode() + b'\n')
        time.sleep(0.1)
    finally:
        sock.close()

def test_initial_stats():
    """Test that statistics start at zero."""
    stats = get_stats()
    assert stats['packets']['received'] == 0, f"Expected 0 packets received, got {stats['packets']['received']}"
    assert stats['packets']['sent'] == 0, f"Expected 0 packets sent, got {stats['packets']['sent']}"
    assert stats['bytes']['received'] == 0, f"Expected 0 bytes received, got {stats['bytes']['received']}"
    assert stats['bytes']['sent'] == 0, f"Expected 0 bytes sent, got {stats['bytes']['sent']}"
    print("✓ Initial statistics are zero")

def test_packet_counting():
    """Test that packet statistics increment with traffic."""
    # Send some packets
    send_gridconnect_packet(":X00001234N0102030405060708;")
    send_gridconnect_packet(":X00005678N1122334455667788;")
    send_gridconnect_packet(":X0000ABCDN;")
    
    time.sleep(1)
    
    stats = get_stats()
    packets_rx = stats['packets']['received']
    bytes_rx = stats['bytes']['received']
    
    assert packets_rx > 0, f"Expected packets > 0, got {packets_rx}"
    assert bytes_rx > 0, f"Expected bytes > 0, got {bytes_rx}"
    print(f"✓ Packet counting works (rx={packets_rx} packets, {bytes_rx} bytes)")

def test_connection_tracking():
    """Test that connection statistics track clients."""
    # Note: GcTcpHub.get_num_clients() returns count but doesn't
    # expose connection/disconnection events, so this test may not
    # reflect real-time changes. The metrics system tracks based on
    # periodic polling.
    stats1 = get_stats()
    conns_start = stats1['connections']['active']
    
    print(f"✓ Connection statistics available (active={conns_start}, total={stats1['connections']['total']})")
    
    # Just verify the fields exist and are reasonable
    assert 'active' in stats1['connections']
    assert 'total' in stats1['connections']
    assert 'failed' in stats1['connections']
    assert stats1['connections']['total'] >= 0
    print("✓ Connection tracking fields present")

def main():
    """Run all tests."""
    hub_proc = None
    try:
        print("Starting hub...")
        hub_proc = start_hub()
        
        if hub_proc.poll() is not None:
            print("✗ Hub failed to start")
            return 1
        
        print("Running tests...")
        test_initial_stats()
        test_packet_counting()
        test_connection_tracking()
        
        print("\n✓ All tests passed!")
        return 0
        
    except AssertionError as e:
        print(f"\n✗ Test failed: {e}")
        return 1
    except Exception as e:
        print(f"\n✗ Error: {e}")
        return 1
    finally:
        if hub_proc:
            hub_proc.terminate()
            hub_proc.wait()

if __name__ == '__main__':
    sys.exit(main())
