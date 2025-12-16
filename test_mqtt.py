#!/usr/bin/env python3
"""
MQTT Test Script for BoltLock
This script publishes lock and unlock commands to test the MQTT implementation.
"""

import json
import time

import paho.mqtt.client as mqtt

# MQTT Configuration (from config.h)
MQTT_BROKER = "alderaan.software-engineering.ie"
MQTT_PORT = 1883
MQTT_TOPIC_COMMAND = "BoltLock/command"
MQTT_TOPIC_STATUS = "BoltLock/status"
MQTT_TOPIC_EVENTS = "BoltLock/events"


def on_connect(client, userdata, flags, rc):
    """Callback when connected to MQTT broker"""
    if rc == 0:
        print(f"✓ Connected to MQTT broker at {MQTT_BROKER}:{MQTT_PORT}")
        # Subscribe to status and events to see responses
        client.subscribe(MQTT_TOPIC_STATUS)
        client.subscribe(MQTT_TOPIC_EVENTS)
        print(f"✓ Subscribed to {MQTT_TOPIC_STATUS}")
        print(f"✓ Subscribed to {MQTT_TOPIC_EVENTS}")
    else:
        print(f"✗ Connection failed with code {rc}")


def on_message(client, userdata, msg):
    """Callback when a message is received"""
    print(f"\n📨 Received on {msg.topic}:")
    try:
        payload = json.loads(msg.payload.decode())
        print(f"   {json.dumps(payload, indent=2)}")
    except Exception:
        print(f"   {msg.payload.decode()}")


def on_publish(client, userdata, mid):
    """Callback when a message is published"""
    print(f"✓ Message published (mid: {mid})")


def publish_command(client, command):
    """Publish a command to the BoltLock"""
    payload = json.dumps({"action": command})
    print(f"\n📤 Publishing command: {command}")
    result = client.publish(MQTT_TOPIC_COMMAND, payload, qos=1)
    if result.rc != mqtt.MQTT_ERR_SUCCESS:
        print(f"✗ Failed to publish: {result.rc}")


def main():
    print("=== BoltLock MQTT Test Script ===\n")

    # Create MQTT client
    client = mqtt.Client(client_id="BoltLock_TestScript")

    # Set callbacks
    client.on_connect = on_connect
    client.on_message = on_message
    client.on_publish = on_publish

    try:
        # Connect to broker
        print(f"Connecting to {MQTT_BROKER}:{MQTT_PORT}...")
        client.connect(MQTT_BROKER, MQTT_PORT, 60)

        # Start network loop in background
        client.loop_start()

        # Wait for connection
        time.sleep(2)

        # Test sequence
        print("\n--- Starting Test Sequence ---")

        # Test 1: Unlock command
        print("\n[Test 1/3] Sending UNLOCK command...")
        publish_command(client, "unlock")
        time.sleep(3)

        # Test 2: Lock command
        print("\n[Test 2/3] Sending LOCK command...")
        publish_command(client, "lock")
        time.sleep(3)

        # Test 3: Unlock again
        print("\n[Test 3/3] Sending UNLOCK command again...")
        publish_command(client, "unlock")
        time.sleep(3)

        print("\n--- Test Sequence Complete ---")
        print("\nKeeping connection open to monitor responses...")
        print("Press Ctrl+C to exit\n")

        # Keep running to receive messages
        while True:
            time.sleep(1)

    except KeyboardInterrupt:
        print("\n\n✓ Test script terminated by user")
    except Exception as e:
        print(f"\n✗ Error: {e}")
    finally:
        client.loop_stop()
        client.disconnect()
        print("✓ Disconnected from MQTT broker")


if __name__ == "__main__":
    main()
