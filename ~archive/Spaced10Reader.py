import csv
rows = []
with open('Figure8.csv') as csvfile:
    readCSV = csv.reader(csvfile, delimiter=',')
    for i, row in enumerate(readCSV):
        if i % 10 == 0:
			rows.append(row)
with open('Figure8_spaced10.csv', 'wb') as csvfile:
	spamwriter = csv.writer(csvfile, delimiter=',')
	for row in rows:
		spamwriter.writerow(row)