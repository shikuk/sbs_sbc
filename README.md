# sbs_sbc
Arduino + SBS + I2C charger bq24xxx autonomous NB battery charger

Hardware: 
	arduino
	bq24725 / bq24725a based charger, cutted out from dead notebook motherboard. Modified battery P-ch mosfet to connect resistive load
	notebook batteries for tests
	board with bq80xx/bq20Zxx from dead notebook battery for cells check (works as I2C analog sensors of voltage/current/temperature)

Purposes:
	standalone check/charge/discharge notebook battery 
	PC based check/charge/discharge notebook battery with graphs
	PC based check/charge/discharge simultaneously 1-4 LION/NiMh/Acid cells with graphs/capacity/Rinternal count
