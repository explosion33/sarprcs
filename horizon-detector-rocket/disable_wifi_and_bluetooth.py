import os
from time import sleep

def disable_wifi_and_bluetooth():
    # Block WIFI and Bluetooth
    os.system('sudo rfkill block wifi')
    os.system('sudo rfkill block bluetooth')
    
    # Check status of WIFI and Bluetooth
    print_out = os.popen('sudo rfkill list').read()

    print_out_cleaned = ''
    for i in print_out:
        if ord(i) == 10:
            print_out_cleaned += ' '
        elif ord(i) == 9:
            print_out_cleaned += ''
        else:
            print_out_cleaned += i
                    
    print(f'Status print out: {print_out_cleaned}')

    # Report WIFI Status
    if "Wireless LAN Soft blocked: yes" in print_out_cleaned:
        wifi_response = "Wifi Disabled."
    elif "Wireless LAN Soft blocked: no":
        wifi_response = "Warning! Wifi is still enabled."
    else:
        wifi_response = "Warning! Wifi status could not be determined."

    # Report Bluetooth Status
    if "Bluetooth Soft blocked: yes" in print_out_cleaned:
        bluetooth_response = "Bluetooth Disabled."
    elif "Bluetooth Soft blocked: no":
        bluetooth_response = "Warning! Bluetooth is still enabled."
    else:
        bluetooth_response = "Warning! Bluetooth status could not be determined."
    
    return wifi_response, bluetooth_response
    
    
    
    
    
    