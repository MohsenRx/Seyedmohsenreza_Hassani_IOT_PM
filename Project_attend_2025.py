#Task 1----------------------------------------------

#**************ATTENTION***************

# All of this and its explanations are in English.

import paho.mqtt.client as mqtt
import RPi.GPIO as GPIO 
import Adafruit_DHT


class device:
    def __init__(self,topic,mqtt_broker='localhost',port=1883):
        self.topic=topic
        self.topic_list=self.topic.split('/')
        
        self.location=self.topic_list[0]
        self.group=self.topic_list[1]
        self.device_type=self.topic_list[2]
        self.name=self.topic_list[3]
        
        self.mqtt_broker=mqtt_broker
        self.port=port
        
        # self.connect_mqtt()
        # self.setup_gpio()
        
    def connect_mqtt(self):
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.connect(self.mqtt_broker, self.port)
        
    def setup_gpio(self):
        if self.device_type == 'lamps':
            GPIO.setup(17, GPIO.OUT)
        elif self.device_type == 'doors':
            GPIO.setup(27, GPIO.OUT)
        elif self.device_type == 'fans':
            GPIO.setup(22, GPIO.OUT)
        elif self.device_type == 'camera':
            GPIO.setup(100, GPIO.OUT)  


class Sensor:
    
    def __init__(self, topic, pin=100):
        self.topic = topic
        self.topic_list = self.topic.split('/')
        self.location = self.topic_list[0]
        self.group = self.topic_list[1]
        self.sensor_type = self.topic_list[2]
        self.name = self.topic_list[3]
        self.pin = pin  

    def read_sensor(self):
        humidity, temperature = Adafruit_DHT.read_retry(Adafruit_DHT.DHT22, self.pin)
        
        if self.sensor_type == 'thermosets':
            return temperature
        else:
            return humidity


#Task 2 & 3--------------------------------------
class admin_panel:
    
    def __init__(self):
        self.groups = {}
        
    def create_group(self, group_name):
        if group_name not in self.groups:
            self.groups[group_name] = []
            print(f'Group {group_name} is created :---->This f-string is printing a message \
                indicating that a group with the specified group_name has been successfully \
                created. The placeholder {group_name} is replaced with the actual name of the group being created.')
        else:
            print('Your group name already exists.')
        
    def add_device_to_group(self, group_name, new_device):
        if group_name in self.groups:
            self.groups[group_name].append(new_device)
        else:
            print(f'Group {group_name} does not exist:----> This f-string prints a message \
                when a group does not exist in the self.groups dictionary. It shows that the \
                group specified by group_name is not found.')
        
    def create_device(self, group_name, device_type, name):
        if group_name in self.groups:
            topic = f'home/{group_name}/{device_type}/{name}'
            new_device = device(topic)
            self.add_device_to_group(group_name, new_device)
        else:
            print(f'Group {group_name} does not exist:----> This f-string prints a message when \
                the specified group (group_name) is not found in self.groups. It informs the user\
                that the group does not exist when trying to create a device under that group..')
        
    def create_multiple_devices(self, group_name, device_type, number_of_devices):
        if group_name in self.groups:
            for i in range(1, number_of_devices + 1):
                topic = f'home/{group_name}/{device_type}/{device_type}{i}'
                new_device = device(topic)
                self.add_device_to_group(group_name, new_device)
                print("Similar to the previous example, this f-string prints a message when the\
                    group specified by group_name does not exist when trying to create multiple \
                    devices under that group.")
        else:
            pass
        
    def turn_on_devices_in_group(self, group_name):
        if group_name in self.groups:
            devices_list = self.groups[group_name]
            for device in devices_list:
                if hasattr(device, 'turn_on'):
                    device.turn_on()
                    print(f'{device.name} in {group_name} turned ON')
        else:
            print(f'Group {group_name} does not exist.')
        
    def turn_off_devices_in_group(self, group_name):
        if group_name in self.groups:
            devices_list = self.groups[group_name]
            for device in devices_list:
                if hasattr(device, 'turn_off'):
                    device.turn_off()
                    print(f'{device.name} in {group_name} turned OFF')
        else:
            print(f'Group {group_name} does not exist.')
        
    def turn_on_all_devices(self):
        for group_name, devices_list in self.groups.items():
            for device in devices_list:
                if hasattr(device, 'turn_on'):
                    device.turn_on()
                    print(f'{device.name} in {group_name} turned ON')

    def turn_off_all_devices(self):
        for group_name, devices_list in self.groups.items():
            for device in devices_list:
                if hasattr(device, 'turn_off'):
                    device.turn_off()
                    print(f'{device.name} in {group_name} turned OFF')
        
    def get_status_in_group(self, group_name):
        if group_name in self.groups:
            devices_list = self.groups[group_name]
            print(f"Status in group '{group_name}':")
            for device in devices_list:
                if hasattr(device, 'status'):
                    print(f"- {device.name} is {device.status.upper()}")
                elif hasattr(device, 'read_sensor'):
                    print(f"- {device.name} (Sensor device)")
        else:
            print(f"Group {group_name} does not exist.")
        
    def get_status_in_device_type(self, device_type):
        found = False
        print(f"Status for device type '{device_type}':")
        for group_name, devices_list in self.groups.items():
            for device in devices_list:
                if hasattr(device, 'device_type') and device.device_type == device_type:
                    found = True
                    print(f"- {device.name} in {group_name} is {device.status.upper()}")
        if not found:
            print(f"No devices of type '{device_type}' found.")
    
    # ============================ sensor functions
    
    def create_sensor(self, group_name, sensor_type, name, pin=100):
        if group_name in self.groups:
            topic = f'home/{group_name}/{sensor_type}/{name}'
            new_sensor = Sensor(topic, pin=pin)
            self.groups[group_name].append(new_sensor)
            print(f"Sensor {name} added to {group_name}")
        else:
            print(f"Group {group_name} does not exist.")
    
    def get_status_sensor_in_group(self, group_name):
        if group_name in self.groups:
            devices_list = self.groups[group_name]
            print(f"Sensor readings in group '{group_name}':")
            found_sensor = False
            for device in devices_list:
                if hasattr(device, 'read_sensor'):
                    reading = device.read_sensor()
                    print(f"- {device.name}: {reading}")
                    found_sensor = True
            if not found_sensor:
                print(f"No sensors found in group '{group_name}'.")
        else:
            print(f"Group {group_name} does not exist.")
