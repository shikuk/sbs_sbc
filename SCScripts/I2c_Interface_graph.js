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

//Data has been received with the main interface.
function dataReceivedSlot(data)
{
	var consoleString = ""
	consoleString = conv.byteArrayToString(data);
	if (consoleString.match("V1")) {
			var value=consoleString.split(" ");
			appendToConsoleBuffer("Get V1=" + value[1]);
		}
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
	plotWidget.setAutoUpdateEnabled(false);
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
	timer.start(10000);
	plotWidget.setAutoUpdateEnabled(true);
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
	graphValues.push(getProcessYValueFromGrap(plotWidgetGraph1Index, xValue));
	graphValues.push(getProcessYValueFromGrap(plotWidgetGraph2Index, xValue));
	graphValues.push(getProcessYValueFromGrap(plotWidgetGraph3Index, xValue));
	
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

function GetCellVolt( cell) 
{
	
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


//Called periodically to add some data to the plot widget
function timeout() 
{
	//add some points to the graphs
	for (var i=1; i<4; i++)
	{
		GetCellVolt(i);

		
		//plotXCounter++;
	}
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

//Create a plot widget and setup them. some example graphs.
var plotWidget = UI_groupBoxPlotContainer.addPlotWidget();
plotWidget.setAxisLabels("x", "y");
plotWidget.showLegend(true);
plotWidget.setInitialAxisRanges(1000, -250, 250);
plotWidget.showHelperElements(true, true, true, true, false, false, true, 100, true);
plotWidget.plotMousePressSignal.connect(plotWindowMousePress);
//create periodically timer which calls the function timeout
var timer = scriptThread.createTimer()
timer.timeoutSignal.connect(timeout);