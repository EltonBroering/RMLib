import serial
import json


ser = serial.Serial('/dev/ttyACM0',115200,timeout=30,rtscts=1)  # open serial port
print("Interface Serial : " + ser.name)         # check which port was really used

vector_deadlines = [12,12,12,120,600]

vector_itens_analyzed = {}

def analyze_data(item):
	if item["TaskState"] == 1:
		if item["TaskIdentifier"] in vector_itens_analyzed:
			vector_itens_analyzed[item["TaskIdentifier"]].append(item)
	else:
		vector_itens_analyzed[item["TaskIdentifier"]] = [item]
	
	for it in vector_itens_analyzed.items():
		if len(it) == 2:
			if(len(it[1]) > 1):
				print(it[1][0])
				print(it[1][1])
				if (it[1][0]["TimeStamp"] - it[1][1]["TimeStamp"]) < vector_deadlines[it[0]-1]:
					print("Task " + str(it[0]) + " Condition: V")
				else:
					if (it[1][0]["TimeStamp"] - it[1][1]["TimeStamp"]) == vector_deadlines[it[0]-1]:
						print("Task " + str(it[0]) + " Condition: U")
					else:
						print("Task " + str(it[0]) + " Condition: F")

while True:
	line = str(ser.readline(),"ISO-8859-1")

	try:
		item = json.loads(line)
		analyze_data(item)
	
	except json.decoder.JSONDecodeError:
		print("Error in json decode -> " + str(line))