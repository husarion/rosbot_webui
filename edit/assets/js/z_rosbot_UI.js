var viewer;
var gridClient;
var mapScale = 5;
var mapZoomSlider;
var mapSizeX = 5;
var mapSizeY = 5;
var mapShiftX;
var mapShiftY;
var exploration_status = "Waiting for trigger";
var previous_status;
var exploration_status_background = "#ffffff";
var max_voltage = 12.6;
var min_voltage = 9.6;
var explorationGoal;
var exploreClient;
var result_subscriber;
var status_subscriber;

var viewPriority = 0;
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

// var sensorLabelFL;
// var sensorLabelFR;
// var sensorLabelRL;
// var sensorLabelRR;

window.onload = function () {
	console.log("onLoad triggered");

	// sensorLabelFL = document.getElementById('sensor-fl');
	// sensorLabelFR = document.getElementById('sensor-fr');
	// sensorLabelRL = document.getElementById('sensor-rl');
	// sensorLabelRR = document.getElementById('sensor-rr');

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

	$(document).on("click", "#video", function () {
	});

	$(document).on("click", "#map", function () {
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
	document.getElementById('video').src = "http://" + location.hostname + ":8082/stream?topic=/camera/rgb/image_raw&type=mjpeg&quality=50"

	mapZoomSlider = document.getElementById("map-zoom");
	mapZoomSlider.oninput = function () {
		setMapScale(mapZoomSlider.value / 10);

		clearTimeout(resize_tout);
		resize_tout = setTimeout(function () {
			gridClient.navigator.setRobotMarker();
		}, 50
		);
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
		mapShiftX = pose.pose.position.x;
		mapShiftY = pose.pose.position.y;
		setMapScale(mapScale);
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
		console.log("FL sensor received")
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


	battery_subscriber.subscribe(function (battery) {
		setBatteryPercentage(100 * (battery.voltage - min_voltage) / (max_voltage - min_voltage));
		lastMsgDate = new Date();
		lastMsgMs = lastMsgDate.getTime();
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

	$(document).on("click", "#explore-button", function () {
		stopTimer();
		startExploration();
	});

	$(document).on("click", "#clear-button", function () {
		cancelExplorationTask();
		clearMap();
	});

	initMap();
	disableStopButton();
	setView();
	
	watchdogTimerInstance = new Timer();
	watchdogTimerInstance.addEventListener('secondTenthsUpdated', watchdogTimer);
	watchdogTimerInstance.start();
};

function watchdogTimer(e) {
	currentMsgDate = new Date();
	currentMsgMs = currentMsgDate.getTime();
	if (currentMsgMs - lastMsgMs > 1000) {
		noMessage = "No messege received since ";
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

	var options = {
		zone: joystickContainer,
		position: { left: 150 + 'px', top: 150 + 'px' },
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

	//sensorLabelRL.style.top = (videoRect.bottom - videoRect.top - 31) + "px";
	//sensorLabelRR.style.top = (videoRect.bottom - videoRect.top - 31) + "px";
}

function setBatteryPercentage(percentage) {
	$("#dynamic").css("width", percentage + "%").attr("aria-valuenow", percentage);
}

$(window).resize(function () {
	setView();
	clearTimeout(resize_tout);
	resize_tout = setTimeout(function () {
		redraw_map();
	}, 100
	);
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

function initMap() {
	var logo = document.getElementById('auto-slide');
	slideRect = logo.getBoundingClientRect();

	viewer = new ROS2D.Viewer({
		divID: 'map',
		width: slideRect.right - slideRect.left,
		height: slideRect.bottom - slideRect.top,
		background: "#7E7E7E"
	});

	document.getElementById("map").style.width = "" + slideRect.right - slideRect.left + "px";
	document.getElementById("map").style.height = "" + slideRect.bottom - slideRect.top + "px";


	gridClient = new NAV2D.OccupancyGridClientNav({
		ros: ros,
		rootObject: viewer.scene,
		viewer: viewer,
		serverName: '/move_base',
		continuous: true
	});
	redraw_map();
}

var resize_tout;

function redraw_map() {
	var map_rect = document.getElementById('map-container');
	mapRect = map_rect.getBoundingClientRect();

	var elem = document.getElementsByTagName('CANVAS');
	if (elem !== undefined) {
		if (elem.length > 0) {
			elem[0].style.width = "" + mapRect.right - mapRect.left - 30 + "px";
			elem[0].style.height = "" + mapRect.bottom - mapRect.top - 10 + "px";
			//elem[0].setAttribute("width", mapRect.right - mapRect.left + "px");
			//elem[0].setAttribute("height", mapRect.bottom - mapRect.top + "px");
		}
	}
	document.getElementById("map").style.width = "" + mapRect.right - mapRect.left - 30 + "px";
	document.getElementById("map").style.height = "" + mapRect.bottom - mapRect.top + "px";
	//document.getElementById("map").setAttribute("width", mapRect.right - mapRect.left + "px");
	//document.getElementById("map").setAttribute("height", mapRect.bottom - mapRect.top + "px");

}

function setMapScale(scale) {
	if (scale !== undefined) {
		mapScale = scale;
	}
	resizeMap();
}

function resizeMap() {
	if (gridClient !== undefined) {
		if (gridClient.client.currentGrid.message !== undefined) {
			mapSizeX = gridClient.client.currentGrid.message.info.width * mapScale / 100;
			mapSizeY = gridClient.client.currentGrid.message.info.height * mapScale / 100;
			viewer.scaleToDimensions(mapSizeX, mapSizeY);
			viewer.shift(mapShiftX - (mapSizeX / 2), mapShiftY - (mapSizeY / 2));
		}
	}
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
	// Use w, s, a, d keys to drive your robot

	// Check if keyboard controller was aready created
	if (teleop == null) {
		// Initialize the teleop.
		teleop = new KEYBOARDTELEOP.Teleop({
			ros: ros,
			topic: '/cmd_vel'
		});
	}

	teleop.scale = 0.25;
}