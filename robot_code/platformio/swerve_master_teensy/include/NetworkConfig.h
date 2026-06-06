#ifndef __NETWORK_CONFIG_H
#define __NETWORK_CONFIG_H

#include <Arduino.h>

// Network configuration for Ethernet telemetry
// Adjust these values to match your network setup

// Teensy MAC address (must be unique on your network)
#define TEENSY_MAC_ADDR {0xA8, 0x61, 0x0A, 0xAE, 0x00, 0xB9}

// IP addresses for Ethernet communication.
// The penny_robot router was NOT serving DHCP (Mac got a 169.254.x self-assigned
// address, Teensy DHCP never got a lease). So use STATIC IPs on a private subnet
// that don't depend on a DHCP server. Set your Mac (on penny_robot_5g, or wired)
// to 192.168.50.2 / 255.255.255.0 to match.
//
// Option A: Static IP (current - works without a DHCP server)
#define TEENSY_IP_ADDR      192, 168, 50, 10     // Teensy static IP
#define RECEIVER_IP_ADDR    192, 168, 50, 2      // Python client computer static IP

// Option B: DHCP (set TEENSY_IP_ADDR to 0,0,0,0). Only works if the router serves DHCP.
// #define TEENSY_IP_ADDR      0, 0, 0, 0

// UDP telemetry port (same port the Python client listens on)
#define TELEMETRY_UDP_PORT  8888

// Debug flag: set to 1 to enable Ethernet debug output
#define NETWORK_DEBUG       1

#endif  // __NETWORK_CONFIG_H
