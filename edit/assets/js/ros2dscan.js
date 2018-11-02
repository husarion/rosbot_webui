/**
 * A shape to draw a sensor_msgs/LaserScan msg
 *
 * @constructor
 * @param options - object with following keys:
 *   * ros - the ROSLIB.Ros connection handle
 *   * topic (optional) - the scan topic to listen
 *   * strokeSize (optional) - the size of the outline
 *   * strokeColor (optional) - the createjs color for the stroke
 */
ROS2D.ScanShape = function (options) {
	var that = this;
	options = options || {};
	this.ros = options.ros;
	this.topic = options.topic || 'scan';
	this.strokeSize = options.strokeSize || 3;
	this.strokeColor = options.strokeColor || createjs.Graphics.getRGB(255, 0, 0);
	this.graphics = new createjs.Graphics();

	var rosTopic = new ROSLIB.Topic({
		ros: this.ros,
		name: this.topic,
		messageType: 'sensor_msgs/LaserScan',
	});

	rosTopic.subscribe(function (message) {
		// check for an old map
		console.log("Scan received");
		that.graphics.clear();
		message.angle_min;
		message.angle_max;
		message.angle_increment;

		for (var i = 1; i < message.ranges.length; ++i) {
			angle = message.angle_min + message.angle_increment * i;
			x = 100 * message.ranges[i] * Math.sin(angle);
			y = 100 * message.ranges[i] * Math.cos(angle);

			that.graphics.beginFill(that.strokeColor);
			that.graphics.drawCircle(400 + x, -400 - y, that.strokeSize);
			createjs.Shape.call(that, that.graphics);
		}
	});
};
ROS2D.ScanShape.prototype.__proto__ = createjs.Shape.prototype;
