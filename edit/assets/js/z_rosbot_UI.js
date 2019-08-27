var mapScale = 5;
var mapZoomSlider;
var exploration_status = "Waiting for trigger";
var previous_status;
var exploration_status_background = "#ffffff";
var max_voltage = 12.6;
var min_voltage = 9.6;
var explorationGoal;
var exploreClient;
var result_subscriber;
var status_subscriber;

var widthThreshold = 500;
var joySize = 100;
var joyWidth = 100;
var joyHeight = 100;
var joyPosY;
var joyPosX;
var manager;
var videoRect;
var videoContainer;

var dbg_str;
var twist;
var ros;
var cmdVel;
var dbg_status;
var reset_publisher;
var clear_publisher;
var bool_reset;
var pose_subscriber;
var rpy_subscriber;
var battery_subscriber;

var sensorSubscriberFL;
var sensorSubscriberFR;
var sensorSubscriberRL;
var sensorSubscriberRR;

var timerInstance;
var watchdogTimerInstance;

var mySwiper;

var teleop;

var lastMsgDate = new Date();
var lastMsgMs = lastMsgDate.getTime();
var currentMsgDate = new Date();
var currentMsgMs = currentMsgDate.getTime();


var wifiSubscriber;

var mapScalePublisher;
var mapScaleMsg;
var batteryText;

window.onload = function () {
	console.log("onLoad triggered");

	dbg_str = new ROSLIB.Message({
		data: "status"
	});

	twist = new ROSLIB.Message({
		linear: {
			x: 0,
			y: 0,
			z: 0
		},
		angular: {
			x: 0,
			y: 0,
			z: 0
		}
	});

	ros = new ROSLIB.Ros({
		url: "ws://" + location.hostname + ":9090"
	});

	cmdVel = new ROSLIB.Topic({
		ros: ros,
		name: '/cmd_vel',
		messageType: 'geometry_msgs/Twist'
	});

	cmdVel.advertise();

	dbg_status = new ROSLIB.Topic({
		ros: ros,
		name: '/dbg_status',
		messageType: 'std_msgs/String'
	});

	reset_publisher = new ROSLIB.Topic({
		ros: ros,
		name: '/reset_odom',
		messageType: 'std_msgs/Bool'
	});

	reset_publisher.advertise();

	clear_publisher = new ROSLIB.Topic({
		ros: ros,
		name: '/reset_map',
		messageType: 'std_msgs/Bool'
	});

	clear_publisher.advertise();

	bool_reset = new ROSLIB.Message({
		data: true
	});

	mapScalePublisher = new ROSLIB.Topic({
		ros: ros,
		name: '/map_zoom',
		messageType: 'std_msgs/Int16'
	});

	mapScaleMsg = new ROSLIB.Message({
		data: 50
	});

	pose_subscriber = new ROSLIB.Topic({
		ros: ros,
		name: '/rosbot_on_map_pose',
		messageType: 'geometry_msgs/PoseStamped'
	});

	rpy_subscriber = new ROSLIB.Topic({
		ros: ros,
		name: '/rpy',
		messageType: 'geometry_msgs/Vector3'
	})

	sensorSubscriberFL = new ROSLIB.Topic({
		ros: ros,
		name: '/range/fl',
		messageType: 'sensor_msgs/Range'
	});
	sensorSubscriberFR = new ROSLIB.Topic({
		ros: ros,
		name: '/range/fr',
		messageType: 'sensor_msgs/Range'
	});
	sensorSubscriberRL = new ROSLIB.Topic({
		ros: ros,
		name: '/range/rl',
		messageType: 'sensor_msgs/Range'
	});
	sensorSubscriberRR = new ROSLIB.Topic({
		ros: ros,
		name: '/range/rr',
		messageType: 'sensor_msgs/Range'
	});

	battery_subscriber = new ROSLIB.Topic({
		ros: ros,
		name: '/battery',
		messageType: 'sensor_msgs/BatteryState'
	});

	wifiSubscriber = new ROSLIB.Topic({
		ros: ros,
		name: '/wifi_status',
		messageType: 'diagnostic_msgs/DiagnosticArray'
	});

	mySwiper = new Swiper('.swiper-container', {
		direction: 'horizontal',
		loop: false,
		slidesPerView: 'auto',
		centeredSlides: false,
		spaceBetween: 0,
		effect: 'slide',
		followFinger: false,
	})

	mySwiper.on('slideChangeTransitionEnd', function () {
		window.dispatchEvent(new Event('resize'));
	});

	videoContainer = document.getElementById('video');
	videoContainer.onload = function () {
		setView()
	};
	document.getElementById('video').src = "http://" + location.hostname + ":8082/stream?topic=/clipping/output&type=mjpeg&quality=100";
	document.getElementById('map-video').src = "http://" + location.hostname + ":8082/stream?topic=/map_image/tile&type=mjpeg&quality=100";

	mapZoomSlider = document.getElementById("map-zoom");
	mapZoomSlider.oninput = function () {
		mapScaleMsg.data = parseInt(mapZoomSlider.value, 10);
		mapScalePublisher.publish(mapScaleMsg);
	}

	ros.on('connection', function () {
		console.log('Connected to websocket server.');
	});

	ros.on('error', function (error) {
		console.log('Error connecting to websocket server: ', error);
		// todo: show big error and ask to reload
	});

	ros.on('close', function () {
		console.log('Connection to websocket server closed.');
	});

	pose_subscriber.subscribe(function (pose) {

		if (document.getElementById("x-pos") !== undefined) {
			document.getElementById("x-pos").innerHTML = pose.pose.position.x.toFixed(2);
		}
		if (document.getElementById("y-pos") !== undefined) {
			document.getElementById("y-pos").innerHTML = pose.pose.position.y.toFixed(2);
		}
		var quaternion = new THREE.Quaternion(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w);
		var rotation = new THREE.Euler().setFromQuaternion(quaternion, "XYZ");
		theta_deg = 57.2957795 * rotation.z;
		document.getElementById("t-pos").innerHTML = theta_deg.toFixed(1) + "°";
		lastMsgDate = new Date();
		lastMsgMs = lastMsgDate.getTime();
	});

	rpy_subscriber.subscribe(function (rpy) {

		if (document.getElementById("roll") !== undefined) {
			document.getElementById("roll").innerHTML = rpy.x.toFixed(2) + "°";
		}
		if (document.getElementById("pitch") !== undefined) {
			document.getElementById("pitch").innerHTML = rpy.y.toFixed(2) + "°";
		}
		if (document.getElementById("yaw") !== undefined) {
			document.getElementById("yaw").innerHTML = rpy.z.toFixed(2) + "°";
		}
		lastMsgDate = new Date();
		lastMsgMs = lastMsgDate.getTime();
	});

	sensorSubscriberFL.subscribe(function (range) {
		if (range.range > range.max_range || range.range < range.min_range) {
			document.getElementById("sensor-label-fl").innerHTML = "Out of range";
		} else {
			document.getElementById("sensor-label-fl").innerHTML = range.range.toFixed(2) + "m";
		}
		lastMsgDate = new Date();
		lastMsgMs = lastMsgDate.getTime();
	});

	sensorSubscriberFR.subscribe(function (range) {
		if (range.range > range.max_range || range.range < range.min_range) {
			document.getElementById("sensor-label-fr").innerHTML = "Out of range";
		} else {
			document.getElementById("sensor-label-fr").innerHTML = range.range.toFixed(2) + "m";
		}
		lastMsgDate = new Date();
		lastMsgMs = lastMsgDate.getTime();
	});

	sensorSubscriberRL.subscribe(function (range) {
		if (range.range > range.max_range || range.range < range.min_range) {
			document.getElementById("sensor-label-rl").innerHTML = "Out of range";
		} else {
			document.getElementById("sensor-label-rl").innerHTML = range.range.toFixed(2) + "m";
		}
		lastMsgDate = new Date();
		lastMsgMs = lastMsgDate.getTime();
	});

	sensorSubscriberRR.subscribe(function (range) {
		if (range.range > range.max_range || range.range < range.min_range) {
			document.getElementById("sensor-label-rr").innerHTML = "Out of range";
		} else {
			document.getElementById("sensor-label-rr").innerHTML = range.range.toFixed(2) + "m";
		}
		lastMsgDate = new Date();
		lastMsgMs = lastMsgDate.getTime();
	});

	batteryText = document.getElementById("batery-percent");
	battery_subscriber.subscribe(function (battery) {
		setBatteryPercentage(100 * (battery.voltage - min_voltage) / (max_voltage - min_voltage), battery.voltage);
		lastMsgDate = new Date();
		lastMsgMs = lastMsgDate.getTime();
	});

	wifiSubscriber.subscribe(function (statuses) {
		updateWifiStatuses(statuses);
	});

	timerInstance = new Timer();

	timerInstance.addEventListener('secondTenthsUpdated', function (e) {
		document.getElementById("exploration-time").innerHTML = timerInstance.getTimeValues().toString();
	});
	timerInstance.addEventListener('started', function (e) {
		document.getElementById("exploration-time").innerHTML = timerInstance.getTimeValues().toString();
	});
	timerInstance.addEventListener('reset', function (e) {
		document.getElementById("exploration-time").innerHTML = timerInstance.getTimeValues().toString();
	});

	clipping_dist = new ROSLIB.Message({
		data: 2.5
	});

	clipping_topic = new ROSLIB.Topic({
		ros: ros,
		name: '/clipping/distance',
		messageType: 'std_msgs/Float32'
	});

	clipping_topic.advertise();

	$(document).on("click", "#explore-button", function () {
		stopTimer();
		startExploration();
	});

	$(document).on("click", "#clear-button", function () {
		cancelExplorationTask();
		clearMap();
	});

	disableStopButton();
	setView();

	var slider = document.getElementById("clipRange");

	slider.oninput = function () {
		updateClippingDistance(this.value / 100);
	}

	watchdogTimerInstance = new Timer();
	watchdogTimerInstance.addEventListener('secondTenthsUpdated', watchdogTimer);
	watchdogTimerInstance.start();
};

function updateClippingDistance(distance) {
	clipping_dist.data = distance;
	clipping_topic.publish(clipping_dist);
	document.getElementById("current-clip-dist").innerHTML = "Move slider to set image clipping distance: " + distance.toString() + "m";
}

function watchdogTimer(e) {
	currentMsgDate = new Date();
	currentMsgMs = currentMsgDate.getTime();
	if (currentMsgMs - lastMsgMs > 1000) {
		noMessage = "No message received since ";
		checkConn = " seconds.\n\rCheck your internet connection and reload this page!"
		$.notify(noMessage.concat(parseInt((currentMsgMs - lastMsgMs) / 1000)).concat(checkConn),
			{
				autoHideDelay: 990,
				position: "top left",
				className: 'warn',
				showDuration: 0,
				hideDuration: 0,
				gap: 2
			});
	}
}

function updateWifiStatuses(wifiStatuses) {
	var wifiStatusP1 = '<div class="text-center mx-1">';
	var wifiStatusP2 = '<img src="assets/img/wifi-';
	var wifiStatusP3 = '.png" class="wifi-status mx-1" alt="wifi-status"></div>';
	var wifiStatusStr;
	var wifiStatusAggregated = "";
	for (i = 0; i < wifiStatuses.status.length; i++) {
		var wifiPercent;
		for (j = 0; j < wifiStatuses.status[i].values.length; j++) {
			if (wifiStatuses.status[i].values[j].key = "percentage") {
				wifiPercent = wifiStatuses.status[i].values[j].value;
			}
		}
		wifiStatusStr = wifiStatusP1 + wifiStatuses.status[i].name + wifiStatusP2 + Math.ceil(wifiPercent / 25) + wifiStatusP3;
		wifiStatusAggregated += wifiStatusStr;
	}
	wifiStatusElem = document.getElementById('wifi-status');
	wifiStatusElem.style.display = "inherit";
	wifiStatusContainer = document.getElementById('wifi-status-container');
	wifiStatusContainer.innerHTML = wifiStatusAggregated;
}

function disableStopButton() {
	document.getElementById("stop-button").disabled = true;
}

function enableStopButton() {
	document.getElementById("stop-button").disabled = false;

}

function clearMap() {
	console.log("Send reuest to clear map");
	clear_publisher.publish(bool_reset);
}

function stopTimer() {
	timerInstance.stop();
}

function startTimer() {
	timerInstance.start();
}

function removeJoystick() {
	joystickContainer = document.getElementById('joystick');
	while (joystickContainer.hasChildNodes()) {
		joystickContainer.removeChild(joystickContainer.childNodes[0]);
	}
	if (!jQuery.isEmptyObject(manager)) {
		manager.destroy();
	}
}

function createJoystick(x, y, w, h) {
	joystickContainer = document.getElementById('joystick');
	var joysticksize;
	if (w < h) {
		joysticksize = w;
	} else {
		joysticksize = h;
	}

	var x_pos = joystickContainer.parentElement.parentElement.offsetWidth / 2;
	var options = {
		zone: joystickContainer,
		position: { left: x_pos + 'px', top: 150 + 'px' },
		mode: 'static',
		size: joysticksize,
		color: '#222222',
		restJoystick: true
	};
	manager = nipplejs.create(options);
	manager.on('move', function (evt, nipple) {
		var direction = nipple.angle.degree - 90;
		if (direction > 180) {
			direction = -(450 - nipple.angle.degree);
		}
		var lin = Math.cos(direction / 57.29) * nipple.distance * 0.005;
		var ang = Math.sin(direction / 57.29) * nipple.distance * 0.05;
		moveAction(lin, ang);
	});
	manager.on('end', function () {
		moveAction(0, 0);
	});
}

function setView() {
	removeJoystick();
	if ($(window).width() > widthThreshold) {
	} else {
	}
	videoRect = document.getElementById("video").getBoundingClientRect();
	joySize = 200;
	if (joySize > ($(window).height() - videoRect.bottom)) {
		if (($(window).height() - videoRect.bottom) < 100) {
		} else {
		}
		joyPosY = ($(window).height() - (joyHeight) - 40);
	} else {
		joyPosY = ((($(window).height() + videoRect.bottom) / 2)) - (joyHeight / 2) - 40;
	}
	joyPosX = (videoRect.right - videoRect.left - joyWidth) / 2;
	createJoystick(0, 0, joyWidth, joyHeight);
	initTeleopKeyboard();
}

function setBatteryPercentage(percentage, voltage) {
	if (percentage < 0) {
		percentage = 0;
	} else if(percentage>100){
		percentage = 100;
	}
	$("#dynamic").css("width", percentage + "%").attr("aria-valuenow", percentage);
	batteryText.innerHTML = "<strong>" + percentage.toFixed(0) + "% [" + voltage.toFixed(2) + " V]</strong>";
}

$(window).resize(function () {
	setView();
});

function resetOdometry() {
	reset_publisher.publish(bool_reset);
}

function moveAction(linear, angular) {
	twist.linear.x = 0;
	twist.linear.y = 0;
	twist.linear.z = 0;
	twist.angular.x = 0;
	twist.angular.y = 0;
	twist.angular.z = 0;
	if (linear !== undefined && angular !== undefined) {
		twist.linear.x = linear;
		twist.angular.z = angular;
	}
	cmdVel.publish(twist);
}

function updateExplorationStatus(status_string, background_color) {
	document.getElementById("task-status").innerHTML = status_string;
}

function extractStatusDesc(status_id) {
	switch (status_id) {
		case 0: // PENDING # The goal has yet to be processed by the action server
			exploration_status = "PENDING";
			exploration_status_background = "#999999";
			break;
		case 1: // ACTIVE # The goal is currently being processed by the action server
			exploration_status = "ACTIVE";
			exploration_status_background = "#ffff00";
			break;
		case 2: // PREEMPTED # The goal received a cancel request after it started executing and has since completed its execution
			exploration_status = "PREEMPTED";
			exploration_status_background = "#999999";
			//stopTimer();
			break;
		case 3: // SUCCEEDED # The goal was achieved successfully by the action server (Terminal State)
			exploration_status = "SUCCEEDED";
			exploration_status_background = "#00ff00";
			stopTimer();
			break;
		case 4: // ABORTED # The goal was aborted during execution by the action server due to some failure
			exploration_status = "ABORTED";
			exploration_status_background = "#ff0000";
			stopTimer();
			break;
		case 5: // REJECTED # The goal was rejected by the action server without being processed, because the goal was unattainable or invalid
			exploration_status = "REJECTED";
			exploration_status_background = "#ff0000";
			stopTimer();
			break;
		case 6: // PREEMPTING # The goal received a cancel request after it started executing and has not yet completed execution
			exploration_status = "PREEMPTING";
			exploration_status_background = "#999999";
			break;
		case 7: // RECALLING # The goal received a cancel request before it started executing, but the action server has not yet confirmed that the goal is canceled
			exploration_status = "RECALLING";
			exploration_status_background = "#999999";
			break;
		case 8: // RECALLED # The goal received a cancel request before it started executing and was successfully cancelled (Terminal State)
			exploration_status = "RECALLED";
			exploration_status_background = "#999999";
			stopTimer();
			break;
		case 9: // LOST # An action client can determine that a goal is LOST. This should not be sent over the wire by an action server
			exploration_status = "LOST";
			exploration_status_background = "#ff0000";
			stopTimer();
			break;
		default:
			exploration_status = "UNKNOWN";
			exploration_status_background = "#ff8000";
			stopTimer();
	}
	previous_status = exploration_status;
}

function cancelExplorationTask() {
	if (explorationGoal !== undefined) {
		console.log("Cancel exploration task");
		explorationGoal.cancel();
		result_subscriber.unsubscribe();
		status_subscriber.unsubscribe();
		$(document).off('click', '#stop-button');
		updateExplorationStatus("CANCELED");
		stopTimer();
	}
	disableStopButton();
}

function startExploration() {
	cancelExplorationTask();
	enableStopButton();

	console.log("Starting exploration task");
	exploreClient = new ROSLIB.ActionClient({
		ros: ros,
		serverName: "/explore_server",
		actionName: "frontier_exploration/ExploreTaskAction",
		timeout: 1000,
		omitFeedback: true,
		omitStatus: true,
		omitResult: true
	});

	explorationGoal = new ROSLIB.Goal({
		actionClient: exploreClient,
		goalMessage: {
			explore_boundary: {
				header: {
					frame_id: "/map"
				},
				polygon: {
					points: [
						{
							x: 30,
							y: 30,
							z: 0
						},
						{
							x: 30,
							y: -30,
							z: 0
						},
						{
							x: -30,
							y: -30,
							z: 0
						},
						{
							x: -30,
							y: 30,
							z: 0
						}
					]
				}
			},
			explore_center: {
				header: {
					frame_id: "/map"
				},
				point: {
					x: 1,
					y: 1,
					z: 0
				}
			}
		}
	});

	explorationGoal.send();

	document.getElementById("current-task").innerHTML = "Explore";

	result_subscriber = new ROSLIB.Topic({
		ros: ros,
		name: '/explore_server/result',
		messageType: 'frontier_exploration/ExploreTaskActionResult'
	});

	result_subscriber.subscribe(function (result) {
		extractStatusDesc(result.status.status);
		updateExplorationStatus(exploration_status, exploration_status_background);
	});

	status_subscriber = new ROSLIB.Topic({
		ros: ros,
		name: '/explore_server/status',
		messageType: 'actionlib_msgs/GoalStatusArray'
	});

	status_subscriber.subscribe(function (status) {
		if (status !== undefined) {
			if (status.status_list.length > 0) {
				extractStatusDesc(status.status_list[status.status_list.length - 1].status);
				updateExplorationStatus(exploration_status, exploration_status_background);
			}
		}
	});

	$(document).on("click", "#stop-button", function () {
		cancelExplorationTask();
	});

	startTimer();
}

function initTeleopKeyboard() {
	// Check if keyboard controller was aready created
	if (teleop == null) {
		teleop = new KEYBOARDTELEOP.Teleop({
			ros: ros,
			topic: '/cmd_vel'
		});
	}
	teleop.scale = 0.25;
}