from pymodbus.client import ModbusTcpClient

client = ModbusTcpClient("192.168.1.32", port=502)
client.connect()
rr = client.read_holding_registers(19, device_id=1)  # 40020 => 19
print(rr.registers)
client.close()
