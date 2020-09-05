using Toybox.Application;
using Toybox.WatchUi;
using Toybox.BluetoothLowEnergy;

class GarminHRBTBroadcastApp extends Application.AppBase {

	var url = "http://192.168.0.252/puthr?";
	var params = {};
	var options = {:method => Communications.HTTP_REQUEST_METHOD_PUT,
				   :headers => {"Content-Type" => Communications.REQUEST_CONTENT_TYPE_URL_ENCODED},
		           :responseType => Communications.HTTP_RESPONSE_CONTENT_TYPE_URL_ENCODED};
	var responseCallback = method(:onReceive);

    function initialize() {
        AppBase.initialize();
        Sensor.setEnabledSensors( [Sensor.SENSOR_HEARTRATE] );
        Sensor.enableSensorEvents( method( :onSensor ) );
    }

    // onStart() is called on application start up
    function onStart(state) {
    }

    // onStop() is called when your application is exiting
    function onStop(state) {
    }

    // Return the initial view of your application here
    function getInitialView() {
        return [ new GarminHRBTBroadcastView(), new GarminHRBTBroadcastDelegate() ];
    }

	function onReceive(responseCode, data) {
    }

	function onSensor(sensorInfo) {
		if (sensorInfo.heartRate != null) {
		    Communications.makeWebRequest(url + sensorInfo.heartRate.toString(), null, options, method(:onReceive));
		}
	}
}
