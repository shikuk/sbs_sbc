/*
In the main function (all code outside a function) only the script initialization code should be placed. 
The working code should be placed in asynchronous function calls (like timer callbacks or data receive callbacks).  
If the main function has been left the script does not stop. To stop a script call scriptThread.stopScript() or 
press the stop button (main or script window).
Example:
************************************************************************************************************
function dataReceived()
{
	//Working code.
	if(workDone)
	{
		scriptThread.stopScript();
	}
}

//Initialization code.
scriptThread.dataReceivedSignal.connect(dataReceived);
************************************************************************************************************

If the worker code shall be placed in the main function then scriptThread.scriptShallExit() should be called 
to check if the script must exit().
Example:
************************************************************************************************************
while(!scriptThread.scriptShallExit())
{
	//Working code.
}
************************************************************************************************************
*/

//Is called if this script shall be exited.
function stopScript() 
{
	scriptThread.appendTextToConsole("I2C script has been stopped");
}

//Is called if the dialog is closed.
function UI_MainWindowFinished(e)
{
	scriptThread.stopScript()
}

function clearConsoleButtonClicked()
{
	UI_scriptConsole.clear();
}

function convertHexStringToByteArray(str) 
{ 
	var result = [];
	
	str = str.replace(/ /gm, "")//remove all spaces;
	
	while (str.length >= 2) 
	{ 
		result.push(parseInt(str.substring(0, 2), 16));
		str = str.substring(2, str.length);
	}
	
	if(str.length == 1)
	{
		result.push(parseInt(str, 16));
	}

	return result;
}

function hexTextLineTextChangedSlot()
{
	var text = this.text();
	text = text.replace(RegExp("[^a-fA-F\\d\\s]"), "");
	text = text.replace(/ /gm, "")//remove all spaces;

	if(this.text() != text)
	{
		this.blockSignals(true);
		this.setText(text);
		this.blockSignals(false);
	}
}

function limitAndFormatHexString(text, withSpaces)
{
	text = text.replace(RegExp("[^a-fA-F\\d\\s]"), "");
	
	var split = text.split(" ");
	text = "";
	
	for(var index = 0; index <  split.length; index++)
	{
		if(split[index].length > 2)
		{
			while(split[index].length > 2)
			{
				text += split[index].slice(0, 2) 
				if(withSpaces)
				{
					text += " ";
				}
				split[index] = split[index].slice(2);
			}
		}
		
		if(split[index].length > 0)
		{
			text += split[index];
			if(withSpaces)
			{
				text += " ";
			}
		}
	}
	
	if(text.indexOf(" ", text.length - 1) != -1)
	{
		text = text.slice(0, text.length - 1)
	}
	
	return text;
}

function updateConsole()
{
	if(consoleBuffer != "")
	{
		consoleTimer.stop();
		var pos = UI_scriptConsole.verticalScrollBarValue();
		UI_scriptConsole.moveTextPositionToEnd();
		UI_scriptConsole.append(consoleBuffer);
		UI_scriptConsole.moveTextPositionToEnd();
		
		consoleBuffer = "";
		consoleTimer.start();
	}
}

function appendToConsoleBuffer(text)
{
	consoleBuffer += text;
}

function GetParams (consoleString) {
	var tempVar = consoleString;

	if (tempVar.indexOf ("get")>=0 ) {
		var startindex=tempVar.search("x");
		tempVar=tempVar.substring(startindex+1,tempVar.length);
		var res = convertHexStringToByteArray(tempVar);
		var outstr = "Get ";
		if (res[0]==0x09)
		outstr += "Charger";
		else if (res[0]==0x0b)
		outstr += "Batt ";
		else 
		outstr += "addr ";

		outstr += sbs_regs[res[1]];

		for (var i=2; i<2+(res.length-2)/2; i++) {
			var lint = res[i+1]<<8 | res[i];
			outstr += " value " + lint +" ";
			
		}
		
		switch (res[1]) {
			case 0x3F: // cell 1
				Vcell1 =  res[3]<<8 | res[2]; 
			break;
			case 0x3E: // cell 2
				Vcell2 =  res[3]<<8 | res[2]; 
			break;
			case 0x3D: // cell 3
				Vcell3 =  res[3]<<8 | res[2]; 
			break;
			case 0x3C: // cell 4
				Vcell4 =  res[3]<<8 | res[2]; 
			break;
			case 0x09: // vPack
				Vpack =  res[3]<<8 | res[2]; 
			break;
			case 0x0A: //  current
				Ibatt =  1000*(res[3]<<8 | res[2]); 
			break;
			case 0x08: // vPack
				Tbatt =  250*((res[3]<<8 | res[2] )-2731)/10; 
			break;
			}
			
		UI_Result.append(outstr);
		UI_I2cBytesToSend.setText(outstr);
	}
}

//Data has been received with the main interface.
function dataReceivedSlot(data)
{
	var consoleString = ""
	consoleString = conv.byteArrayToString(data);
	GetParams (consoleString);
	appendToConsoleBuffer("<span style=\"color:#00ff00;\">" + UI_scriptConsole.replaceNonHtmlChars(consoleString) + "</span>");
}

function executeI2cSlot()
{
	
	var reg;
	var slaveAddress;
	var numberOfBytesToRead;
	var dataToSend = Array();
	
	var tmpText = UI_I2cAddress.text();
	for(var i = UI_I2cAddress.text().length; i < 2; i++)
	{
		tmpText = "0" + tmpText;
	}
	UI_I2cAddress.setText(tmpText);
	
	var tmpText = UI_I2cReg.text();
	for(var i = UI_I2cReg.text().length; i < 2; i++)
	{
		tmpText = "0" + tmpText;
	}
	UI_I2cReg.setText(tmpText);
	
	slaveAddress = parseInt(UI_I2cAddress.text().slice(0,2), 16) & 0xff;	
	reg = parseInt(UI_I2cReg.text().slice(0, 2), 16) & 0xff;
	numberOfBytesToRead = parseInt(UI_I2cNumberOfBytesToRead.value());
	UI_I2cBytesToSend.setText(limitAndFormatHexString(UI_I2cBytesToSend.text(), true));
	
	
	var tmpText = "iicc";
	tmpText = tmpText+(UI_RB_Write.isChecked()==true ? "1" : "0")+UI_I2cAddress.text()+UI_I2cReg.text()+UI_I2cNumberOfBytesToRead.value();
	
	if(scriptInf.sendString(tmpText))
	{
		UI_scriptConsole.append("execute I2C send: data= " + tmpText);
	}
	else
	{
		UI_scriptConsole.append("execute I2C failed");
	}
	
	
}

function GetSBSDump() 
{
	if(scriptInf.sendString("dump"))
	{
		UI_scriptConsole.append("Call Smart Battery data dump");
	}
	else
	{
		UI_scriptConsole.append("execute Smart Battery data dump failed");
	}
}

function StopChargeDischarge() 
{
	if(scriptInf.sendString("stop"))
	{
		UI_scriptConsole.append("Stop charge/discharge");
	}
	else
	{
		UI_scriptConsole.append("execute Stop charge/discharge failed");
	}
	timer.stop();
	plottimer.stop();
	plotWindow1.setAutoUpdateEnabled(false);
}

function GetI2CDevices() 
{
	if(scriptInf.sendString("devs"))
	{
		UI_scriptConsole.append("Search I2C devices");
	}
	else
	{
		UI_scriptConsole.append("execute Search I2C devices  failed");
	}
}

function GetI2CDevregs() 
{
	var tmpText = UI_RegsAddr.text();
	for(var i = UI_RegsAddr.text().length; i < 2; i++)
	{
		tmpText = "0" + tmpText;
	}
	UI_RegsAddr.setText(tmpText);

	tmpText = "regs"+tmpText;
	if(scriptInf.sendString(tmpText))
	{
		UI_scriptConsole.append("Scan existing regs at  I2C device" +  UI_RegsAddr.text() );
	}
	else
	{
		UI_scriptConsole.append("execute Scan existing regs failed");
	}
}

function SetVolt() 
{
	var tmpText = UI_Volts.text();
	tmpText = "volt"+tmpText;
	
	if(scriptInf.sendString(tmpText))
	{
		UI_scriptConsole.append("Set charger voltage" +  UI_Volts.text() );
	}
	else
	{
		UI_scriptConsole.append("execute Set charger voltage failed");
	}
}

function SetCurrent() 
{
	var tmpText = UI_Current.text();

	tmpText = "curr"+tmpText+"\r\n";
	
	if(scriptInf.sendString(tmpText))
	{
		UI_scriptConsole.append("Set charge/discharge current" +  UI_Current.text() );
	}
	else
	{
		UI_scriptConsole.append("executeScan existing regs failed");
	}
}

function GetManuf() 
{
	var tmpText = UI_ManufReg.text();

	for(var i = UI_ManufReg.text().length; i < 4; i++)
	{
		tmpText = "0" + tmpText;
	}
	UI_ManufReg.setText(tmpText);
	 tmpText = UI_ManufReg.text();

	reg = parseInt(UI_ManufReg.text().slice(2, 4), 16) & 0xff;
	reg = reg<<8 &  parseInt(UI_ManufReg.text().slice(0, 2), 16) & 0xff;

	tmpText = "iicm "+tmpText;
	
	if(scriptInf.sendString(tmpText))
	{
		UI_scriptConsole.append("Set manuf reg value" +  UI_Current.text() );
	}
	else
	{
		UI_scriptConsole.append("execute manuf reg value failed");
	}
}

//The plot window is closed.
function plotWindowClosedSlot()
{
	plottimer.stop();
	plotWindow1.setAutoUpdateEnabled(false);
}

//Is called if the user clicks the button.
function clearButtonPressed()
{
	//Clear all data with clearGraphs.
	plotWindow1.clearGraphs();
	plotWindow2.clearGraphs();
	
}

//Is called if the user clicks on the plot in window 1.
function plotWindowMousePress(xValue, yValue, button)
{
	scriptThread.appendTextToConsole('plotWindowMousePress: ' + xValue + ", " + yValue + ", " + button);
}

//Is called if the user clicks on the plot in window 2.
function plotWindow2MousePress(xValue, yValue, button)
{
	scriptThread.appendTextToConsole('plotWindow2MousePress: ' + xValue + ", " + yValue + ", " + button);
}


function plotWindowShow() {
	plotWindow1.show();
	plotWindow1.clearButtonPressedSignal.connect(clearButtonPressed)
	plotWindow1.closedSignal.connect(plotWindowClosedSlot)
	plotWindow1.setMaxDataPointsPerGraph(10000);
	plotWindow1.setUpdateInterval(4000);
	//plotWindow1.plotMousePressSignal.connect(plotWindowMousePress)
	
}

function StartCharge() 
{
	var tmpText = UI_Current.text();
	if (tmpText.length==0) {
		tmpText="";
		scriptThread.messageBox("Critical", "Error", "Set required Charge current");
	}
	else {
		tmpText = "schg"+tmpText+"\r\n";
		if(scriptInf.sendString(tmpText))
		{
			UI_scriptConsole.append("Start Charge with current" +  UI_Current.text() + " mA" );
		}
		else
		{
			UI_scriptConsole.append("execute charge failed");
		}
	}
	quene=0;
	timer.start(777);
	plottimer.start(5000);
	startTime=Date.now();
	plotWindow1.setAutoUpdateEnabled(true);
	plotWindowShow() ;
	row=0;
	adjustTableColmnWidth();
}

function StartDischarge() 
{
	var tmpText = UI_Current.text();
	if (tmpText.length==0) {
		tmpText="";
		scriptThread.messageBox("Critical", "Error", "Set required disCharge current");
	}
	else {
		tmpText = "dchg"+tmpText+"\r\n";
		if(scriptInf.sendString(tmpText))
		{
			UI_scriptConsole.append("Start DisCharge with current " +  UI_Current.text() + " mA" );
		}
		else
		{
			UI_scriptConsole.append("execute DisCharge failed");
		}
	}
}

//Is called if the user clicks on the plot.
function plotWindowMousePress(xValue, yValue, button)
{
	if ((button != 1) && (button != 2))
	return;

	var column = button - 1;
	
	var graphValues = [];
	graphValues.push(xValue.toFixed(2));
	graphValues.push(getProcessYValueFromGrap(plotCell1, xValue));
	graphValues.push(getProcessYValueFromGrap(plotCell2, xValue));
	graphValues.push(getProcessYValueFromGrap(plotCell3, xValue));
	graphValues.push(getProcessYValueFromGrap(plotCurr, xValue));
	graphValues.push(getProcessYValueFromGrap(plotTpack, xValue));
	graphValues.push(getProcessYValueFromGrap(plotVpack, xValue));
	
	for (var row=0; row<graphValues.length; row++)
	{
		var textOld = UI_tableWidgetMarkers.getText(row, column);
		var textNew = graphValues[row];
		
		var color = (textOld == textNew) ? "black" : "red";
		
		UI_tableWidgetMarkers.setText(row, column, textNew);
		UI_tableWidgetMarkers.setCellForegroundColor(color, row, column);
		
		// calculate difference beetween second and first point
		var first = parseFloat(UI_tableWidgetMarkers.getText(row, 0));
		var second = parseFloat(UI_tableWidgetMarkers.getText(row, 1));
		
		var textOld = UI_tableWidgetMarkers.getText(row, 2);
		var textNew = (second - first).toFixed(2);
		
		color = (textOld == textNew) ? "black" : "red";
		
		UI_tableWidgetMarkers.setText(row, 2, textNew);
		UI_tableWidgetMarkers.setCellForegroundColor(color, row, 2);
	}
	
	scriptThread.appendTextToConsole('plotWidgetMousePress: ' + xValue + ", " + yValue + ", " + button);
}

function GetCellVolt( cell)  {
	
	var tmpText = "iicc00B";
	if (cell ==1) 
	tmpText += "3F";
	if (cell ==2) 
	tmpText += "3E";
	if (cell ==3) 
	tmpText += "3D";
	tmpText += "2\r\n";

	scriptInf.sendString(tmpText);

}

function GetCurrent() {
	var tmpText = "iicc00B0A2\r\n";
	scriptInf.sendString(tmpText);
}

function GetVpack() {
	var tmpText = "iicc00B092\r\n";
	scriptInf.sendString(tmpText);
}

function GetTemp() {
	var tmpText = "iicc00B082\r\n";
	scriptInf.sendString(tmpText);
}

//Called periodically to add some data to the plot widget
function timeout() 
{
	switch (quene) {
		case 0:
			GetCurrent();
			break;
		case 1:
			GetVpack();
			break;
		case 2:
			GetTemp();
			break;
		case 3:
		GetCellVolt(1);
			break;
		case 4:
		GetCellVolt(2);
		break;
		case 5:
			GetCellVolt(3);
		break;
		case 6:
			GetCellVolt(4);
		break;				
	}
	quene++;
	if (quene==7) 
			quene=0;
}

function plotAdd() {
	var x = (Date.now()-startTime)/1000;
		plotWindow1.addDataToGraph(plotVpack,  x, Vpack);
		plotWindow1.addDataToGraph(plotTpack,  x, Ibatt);
		plotWindow1.addDataToGraph(plotCurr,  x, Tbatt);
		plotWindow1.addDataToGraph(plotCell1,  x, Vcell1);
		plotWindow1.addDataToGraph(plotCell2,  x, Vcell2);
		plotWindow1.addDataToGraph(plotCell3,  x, Vcell3);
// var tmpString = Vpack.toString()+","+Ibatt.toString()+","+Tbatt.toString()+","+ Vcell1.toString()+","+ Vcell2.toString()+","+ Vcell3.toString();

UI_tableWidget.insertRowWithContent (row, Array (Vpack.toString(), Ibatt.toString(), Tbatt.toString(), Vcell1.toString(), Vcell2.toString(), Vcell3.toString()), Array ("black", "black", "black" , "black", "black", "black"), Array ("white", "white", "white", "white", "white", "white"));
	row++;
	adjustTableColmnWidth();
//	for (var i=0; i<6; i++) 
//		UI_tableWidget.resizeColumnToContents(i);
}

//Adjust the width of the right column, so that all columns fit in the complete table.
function adjustTableColmnWidth()
{
	var verticalScrollBarWidth = 0;
	if(UI_tableWidget.isVerticalScrollBarVisible())
	{
		verticalScrollBarWidth = UI_tableWidget.verticalScrollBarWidth();
	}
	var columnWidth = (UI_tableWidget.width() -
                           (2 * UI_tableWidget.frameWidth()
                           + UI_tableWidget.verticalHeaderWidth()
						   + verticalScrollBarWidth)) /6;
	for (var i=0; i<6; i++) 
			UI_tableWidget.setColumnWidth(i,columnWidth);
}

scriptThread.appendTextToConsole('I2C script has started');
UI_MainWindow.finishedSignal.connect(UI_MainWindowFinished);

var consoleBuffer = ""
var consoleTimer = scriptThread.createTimer()
consoleTimer.timeoutSignal.connect(updateConsole);
consoleTimer.start(200);
scriptInf.dataReceivedSignal.connect(dataReceivedSlot);
UI_clearConsoleButton.clickedSignal.connect(clearConsoleButtonClicked);
UI_SendButton.clickedSignal.connect(executeI2cSlot);

UI_I2cBytesToSend.textChangedSignal.connect(UI_I2cBytesToSend, hexTextLineTextChangedSlot);	 
UI_I2cAddress.textChangedSignal.connect(UI_I2cAddress, hexTextLineTextChangedSlot);
UI_I2cReg.textChangedSignal.connect(UI_I2cReg, hexTextLineTextChangedSlot);
UI_Close.clickedSignal.connect(UI_MainWindowFinished);

UI_Dump.clickedSignal.connect(GetSBSDump);
UI_ScanDev.clickedSignal.connect(GetI2CDevices);
UI_ScanRegs.clickedSignal.connect(GetI2CDevregs);
UI_SetVolt.clickedSignal.connect(SetVolt);
UI_SetCurrent.clickedSignal.connect(SetCurrent);
UI_StartChg.clickedSignal.connect(StartCharge);
UI_StartDisch.clickedSignal.connect(StartDischarge);
UI_Stop.clickedSignal.connect(StopChargeDischarge);

UI_GetManuf.clickedSignal.connect(GetManuf);

UI_tableWidget.setBackgroundColor("black");
UI_tableWidget.setColumnCount(6);
UI_tableWidget.setColumnWidth(1, 8);
//Create a plot widget and setup them. some example graphs.
var plotWindow1 = scriptThread.createPlotWindow();
plotWindow1.setAxisLabels("time", "V/A");
plotWindow1.showLegend(false);
plotWindow1.setInitialAxisRanges(1800, 0, 15000);
plotWindow1.showHelperElements(true, true, true, true, false, false, true, 100, true);
plotWindow1.plotMousePressSignal.connect(plotWindowMousePress);
var plotVpack = plotWindow1.addGraph("darkMagenta", "solid", "Vpack");
plotWindow1.setLineStyle(plotVpack, "Line");
var plotTpack = plotWindow1.addGraph("red", "solid", "Temperature");
plotWindow1.setLineStyle(plotTpack, "Line");
var plotCurr = plotWindow1.addGraph("blue", "solid", "Current");
plotWindow1.setLineStyle(plotCurr, "Line");
plotWindow1.setLineWidth(plotCurr, 3);
var plotCell1 = plotWindow1.addGraph("lightGray", "solid", "VCell1");
plotWindow1.setLineStyle(plotCell1, "Line");
var plotCell2 = plotWindow1.addGraph("darkGray", "solid", "VCell2");
plotWindow1.setLineStyle(plotCell2, "Line");
var plotCell3 = plotWindow1.addGraph("gray", "solid", "VCell3");
plotWindow1.setLineStyle(plotCell3, "Line");

//create periodically timer which calls the function timeout
var timer = scriptThread.createTimer()
timer.timeoutSignal.connect(timeout);
var plottimer = scriptThread.createTimer()
plottimer.timeoutSignal.connect(plotAdd);
var quene=0;
var Vcell1=0;
var Vcell2=0;
var Vcell3=0;
var Vcell4=0;
var Vpack=0;
var Ibatt=0;
var Tbatt=0;
var startTime;

var row;
var sbs_regs =["ManufacturerAccess","RemainingCapacityAlarm","RemainingTimeAlarm","BatteryMode",
"AtRate",
"AtRateTimeToFul","AtRateTimeToEmpty","AtRateOK",
"Temperature","Voltage","Current","AverageCurrent",
"MaxError",
"RelativeStateOfCharge","AbsoluteStateOfCharge",
"RemainingCapacity",
"FullChargeCapacity",
"RunTimeToEmpty",
"AverageTimeToEmpty",
"AverageTimeToFull",
"ChargingCurrent",
"ChargingVoltage",
"BatteryStatus",
"CycleCount",
"DesignCapacity",
"DesignVoltage",
"SpecificationInfo",
"ManufactureDate",
"SerialNumber",
"Not Exist",
"Not Exist",
"Not Exist",
"ManufacturerName",
"DeviceName String",
"DeviceChemistry String",
"ManufacturerData String",
"Not Exist",
"Not Exist",
"Not Exist",
"Not Exist",
"Not Exist",
"Not Exist",
"Not Exist",
"Not Exist",
"Not Exist",
"Not Exist",
"Not Exist",
"Authenticate",
"Not Exist",
"Not Exist",
"Not Exist",
"Not Exist",
"Not Exist",
"Not Exist",
"Not Exist",
"Not Exist",
"Not Exist",
"Not Exist",
"Not Exist",
"Not Exist",
"CellVoltage4",
"CellVoltage3",
"CellVoltage2",
"CellVoltage1"
];
