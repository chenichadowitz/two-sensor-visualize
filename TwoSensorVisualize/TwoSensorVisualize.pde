import shapes3d.utils.*;
import saito.objloader.*;
import processing.serial.*;
import java.nio.ByteBuffer;
import java.util.Arrays;

JSONObject config;

// Variable to manage manual mouse-based rotation
float rotX = 0f;

// One serial port per sensor
Serial[] ports = new Serial[2];

// One flag for each sensor to determine if we've synced the data streams
boolean[] first = {true, true};

// One Rotation object per sensor to keep track of each orientation (default: no rotation)
Rot[] r = {new Rot(new PVector(1,0,0), 0), new Rot(new PVector(1,0,0), 0)};

// One PVector per sensor to keep track of each rotation axis 
PVector[] axis = {new PVector(),new PVector()};
// One float per sensor to keep track of each rotation angle
float[] angle = {0,0};

// An OBJModel object for each piece of the model (body, upper arm, forearm)
OBJModel body, upperArm, foreArm;

// Scale parameter to make sure the model(s) are easily visible. Default: 40
float scaleParam = 40;

// Flag to determine if we should display the body model or just the arm
boolean drawBody = false;

// A BoundingBox object for the upper arm and forearm to help determine the joint positions
BoundingBox upperArmBox, foreArmBox;

// Flag to easily switch between bluetooth mode (connect=true) and static mode (connect=false)
boolean connect = true;

// Flag to determine if we should show the COM ports or not
boolean showPorts = false;

/* Set these to the indices of the serial (COM) ports that match the ports of each sensor
 * Each sensor will create two ports, one each for input and output. If the sensors are the only
 * devices that created serial ports, there will only be 4 ports and these indices will be 0-3.
 * You may need to do trial and error to figure out which ports match. Generally, if you connect
 * the upper arm sensor to your computer first, the index will be 0, and then once you've connected
 * the forearm sensor, it will have an index of 2.
 */
int upperArmSensorIndex;
int foreArmSensorIndex;

// Set up the program
void setup() {
  // Resolution by default is 1000x800, using P3D which tries to use OpenGL if possible
  size(1000, 800, P3D);
  
  // Disable drawing the line-strokes
  noStroke();
  
  // Instantiate the three OBJModel objects with the appropriate .obj files
  // If you need to use an .obj that has not been triangulated, try using QUAD instead of TRIANGLE
  body = new OBJModel(this, "wooden_body.obj", TRIANGLE);
  upperArm = new OBJModel(this, "wooden_shoulder.obj", TRIANGLE);
  foreArm = new OBJModel(this, "wooden_forearm.obj", TRIANGLE);
  
  body.scale(scaleParam);
  upperArm.scale(scaleParam);
  foreArm.scale(scaleParam);
  
  // Instantiate the BoundingBox objects for the two arm segments
  upperArmBox = new BoundingBox(this, upperArm);
  foreArmBox = new BoundingBox(this, foreArm);
  
  // List all the available serial ports:
  println("Available serial (COM) ports:");
  for(String s : Serial.list()) println(s);
  
  // Connect to the two serial ports
  if(connect) {
    // Exit if no serial ports found
    if(Serial.list().length == 0) {
      println("No serial ports found!");
      exit();
    } else {
      // Instantiate the JSON object to read the configuration data for the bluetooth sensors
      config = loadJSONObject("config.json");
      // Load the config data
      upperArmSensorIndex = config.getInt("upperArmSensorIndex");
      foreArmSensorIndex = config.getInt("foreArmSensorIndex");
      // No config values set? Exit.
      if(upperArmSensorIndex < 0 || foreArmSensorIndex < 0) {
        showPorts = true;
      } else {
        // Get the port names for each sensor (required to connect to them)
        String upperArmPortName = Serial.list()[upperArmSensorIndex];
        String foreArmPortName = Serial.list()[foreArmSensorIndex];
        
        // Connect to the two serial ports with a baudrate of 115200
        // To help synchronize the data streams, buffer the incoming data until the first '$'
        // as this marks the beginning of a data packet
        ports[0] = new Serial(this, upperArmPortName, 115200);
        ports[0].bufferUntil('$');
        ports[1] = new Serial(this, foreArmPortName, 115200);
        ports[1].bufferUntil('$');
    
        println("Connected and buffering!");
      }
    }
  }
}

// Runs each frame
void draw() {
  // Set the background color to grey
  background(129);
  // Use default lighting
  lights();
  
  if(showPorts) {
    text("Available serial (COM) ports:", 5, 15);
    for(int i=0; i<Serial.list().length; i++) {
      text(i+": "+Serial.list()[i], 5, 30+i*15);
    }
  } else {
    pushMatrix();
      // Translate to a good view point so that the model is in sight
      translate(width/2.0, height/2.0+125, 0);
      
      // Allow the mouse to rotate the scene around the vertical axis
      rotateY(rotX);
  
      if(drawBody) body.draw();
      
      // Get the width/height/depth of the upper arm bounding box
      PVector whd = upperArmBox.getWHD();
      // Get the center of the upper arm bounding box
      PVector center = upperArmBox.getCenter();
  
      // Translate the rotation point to the center of the upper arm bounding box
      translate(center.x, center.y, center.z);
      // Translate the rotation point to the shoulder joint
      translate(whd.x/4.0, -whd.y/2.0, 0);
      // Rotate about the Z-axis so that by default the arm is hanging down
      rotateZ(PI/2.0);
      // Rotate the upper arm by the amount provided by the sensor
      // See the docs for how the axes convert from the sensor frame of reference to this one 
      rotate(angle[0], -axis[0].z, -axis[0].y, axis[0].x);
      // Translate back to the center of the scene 
      translate(-center.x-whd.x/4.0, -center.y+whd.y/2.0, 0);
      // Draw the upper arm
      upperArm.draw();
      
      pushMatrix();
        // Get the width/height/depth and center of the forearm bounding box
        whd = foreArmBox.getWHD();
        center = foreArmBox.getCenter();
  
        // Translate the rotation point to the top-center of the forearm bounding box      
        translate(center.x, center.y-whd.y/2.0, center.z);
        
        // Since both sensors are using world-frame orientations, 
        // 'undo' the effects from the upper arm sensor before applying the change from the forearm sensor
        // Again, see docs for axes conversion
        rotate(-angle[0], -axis[0].z, -axis[0].y, axis[0].x);
        rotate(angle[1], -axis[1].z, -axis[1].y, axis[1].x);
        // Translate back to the center of the scene
        translate(-center.x, -center.y+whd.y/2.0, -center.z);
        // Draw the forearm
        foreArm.draw();
      popMatrix();
    popMatrix();
  }
}

/* Helper method to process the data stream
 * @param byte[] rs: a byte array read from the data stream which is 23 bytes long
 * @param int id: indicates which sensor this data packet came from (0 or 1)
 */
void processQuat(byte[] rs, int id) {
  // Quaternions come as (w, x, y, z)
  // Each component of the quaternion is 4 bytes long
  // Extract each component into a float
  float q0 = ByteBuffer.wrap( subset(rs, 2, 4) ).getInt() * 1.0 / (1<<30);
  float q1 = ByteBuffer.wrap( subset(rs, 6, 4) ).getInt() * 1.0 / (1<<30);
  float q2 = ByteBuffer.wrap( subset(rs, 10, 4) ).getInt() * 1.0 / (1<<30);
  float q3 = ByteBuffer.wrap( subset(rs, 14, 4) ).getInt() * 1.0 / (1<<30);
  
  // Create a new Rotation object from this quaternion and save it to the Rotation array
  r[id] = new Rot(q0, q1, q2, q3, true);
  
  // Extract the angle-axis representation of this rotation for easy use
  axis[id] = r[id].getAxis();
  angle[id] = r[id].getAngle();
}

/* Helper method to parse the data stream and determine 
 * if this is a quaternion or some other kind of data (currently only processes quaternion packets)
 * @param byte[] rs: a byte array of 23 bytes long where the first byte is an indicator of the packet type
 * @param int id: indicates which sensor this packet came from (0 or 1)
 */
void parseResponse(byte[] rs, int id) {
  // If the first byte is equal to 2, this is a quaternion
  if(rs[0] == 2) {
    processQuat(rs, id); 
  }
}

/* Callback function for the serial ports called when there is buffered data available
 * @param Serial port: the port that has buffered data available (automatically called)
 */
void serialEvent(Serial port) {
  // portNum will be 0 or 1 after determining which port received this data
  int portNum = -1;
  
  // If we have connected to the first serial port and it is the same port as the one with data
  if(ports[0] != null && equalsPort(port, ports[0])) {
    // If this is the first time we've read data from the port, clear the buffered data and continue (helps with synchronization)
    if(first[0]) {
      first[0] = false;
      port.clear();
      return;
    } else {
      // Else, save the port number
      portNum = 0;
    }
  }
  // If we have connected to the second serial port and it is the same port as the one with data 
  else if(ports[1] != null && equalsPort(port, ports[1])) {
    // If this is the first time we've read data from this port, clear the buffered data and continue
    if(first[1]) {
      first[1] = false;
      port.clear();
      return;
    } else {
      // Else, save the port number
      portNum = 1;
    }
  }
  // Create a byte array buffer to read the sensor data into
  byte[] buf = new byte[23];
  int bytesread = port.readBytes(buf);
  // If the last byte is a '$', that means we read an entire packet
  // since the data is being buffered until a '$', so parse the packet
  if(buf[22] == '$') {
    parseResponse(buf, portNum);
  }
}

/* Triggered when a keyboard button is pressed */
void keyPressed() {
  if(key == 't') drawBody = !drawBody;
  else {
    if(key == ESC) exit();
    else {
      if(key != CODED && !showPorts) {
        println("Sending (to first sensor): "+key);
        // Send the keystroke out:
        ports[0].write(key);
        println("Sent!");
      }
    }
  }
}

/* Helper method to compare Serial objects
 * @param Serial first: the first Serial object to compare
 * @param Serial second: the second Serial object to compare
 * @returns boolean: true if the names are equal, false otherwise
 */
boolean equalsPort(Serial first, Serial second) {
  return first.port.getName().equals(second.port.getName());
}

/* Triggered when a mouse button is held down while moving the mouse */
void mouseDragged() {
  // Compute the difference between the previous mouse location and the current location
  float x = (mouseX-pmouseX);
  if(mouseButton == LEFT) {
    // Uses the amount of movement to rotate the scene
    rotX += x * 0.01;
  }
}
