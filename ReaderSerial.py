import serial
import json
import time



while True:
	try:
		ser = serial.Serial('/dev/ttyACM0',576000,timeout=30,rtscts=1)  # open serial port
		print("Interface Serial : " + ser.name)         # check which port was really used
		break
	except:
		print("Error in open serial")
		time.sleep(0.01)

vector_deadlines = [12,12,12,120,600]
vector_period = [12,12,12,120,600]
vector_wcet = [2,3,4,4,135]


vector_itens_analyzed = {}

def analyze_data(item):
	if item["TaskState"] == 1:
		if item["TaskIdentifier"] in vector_itens_analyzed:
			vector_itens_analyzed[item["TaskIdentifier"]].append(item)
	else:
		vector_itens_analyzed[item["TaskIdentifier"]] = [item]



	
	if item["TaskIdentifier"] in vector_itens_analyzed:
		if len(vector_itens_analyzed[item["TaskIdentifier"]]) == 2:
			it = vector_itens_analyzed.pop(item["TaskIdentifier"])
			if(len(it) > 1):
				print(it)
				
				if (it[1]["TimeStamp"] - it[0]["TimeStamp"]) <= vector_wcet[it[0]["TaskIdentifier"]-1]:
					print("Task " + str(it[0]["TaskIdentifier"]) + " Condition WCET: T - WCET Time: " + str((it[1]["TimeStamp"] - it[0]["TimeStamp"])))
				else:
					print("Task " + str(it[0]["TaskIdentifier"]) + " Condition WCET: F - WCET Time: " + str((it[1]["TimeStamp"] - it[0]["TimeStamp"])))

				if (vector_period[it[0]["TaskIdentifier"]-1] == 0):
					if((vector_wcet[it[1]["TaskIdentifier"]-1]) >= it[1]["TimeStamp"]):
						print("Task " + str(it[0]["TaskIdentifier"]) + " Condition DeadLine: T")
					else:
						print("Task " + str(it[0]["TaskIdentifier"]) + " Condition DeadLine: F")
				else:
					if ((it[1]["TaskCounter"]*vector_period[it[1]["TaskIdentifier"]-1]) >= it[1]["TimeStamp"]):
						print("Task " + str(it[0]["TaskIdentifier"]) + " Condition DeadLine: T")
					else:
						print("Task " + str(it[0]["TaskIdentifier"]) + " Condition DeadLine: F")
				print("Task " + str(it[0]["TaskIdentifier"]) + " TimeStamp: " + str(it[1]["TimeStamp"]) + " DeadLine: " + str(it[1]["TaskCounter"]*vector_deadlines[it[1]["TaskIdentifier"]-1]))


def process_optimize_data(item):
	dict_tmp = {}
	items = item.strip().split("-")
	dict_tmp["TaskIdentifier"] 	= int(items[0].split("I")[1])
	dict_tmp["TaskState"] 		= int(items[1].split("S")[1])
	dict_tmp["TimeStamp"] 		= int(items[2].split("T")[1])
	dict_tmp["TaskCounter"] 	= int(items[3].split("C")[1])
	return dict_tmp

def process_data(item):
	if(item[0] == "I"):
		return process_optimize_data(item)
	else:
		return json.loads(item)


while True:
	try:
		line = str(ser.readline(),"ISO-8859-1")
		item = process_data(line)
		analyze_data(item)
	
	except json.decoder.JSONDecodeError:
		print("Error in json decode -> " + str(line))
