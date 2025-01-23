// Mostly taken from https://github.com/Teddy-Mengistie/ArmTraj
// and modified slightly to fit our use-case, thanks!

import controlP5.*;
import java.util.*;

// UI objects
ControlP5 cp5;
Textfield fileName;
Textfield Q1D;
Textfield LD;
Textfield Q1DD;
Textfield LDD;
ScrollableList dropdownA;
ScrollableList dropdownB;
Textfield Q1;
Textfield L;
Textfield WAYPOINT_NAME;

PImage reefImage;

// Saved waypoints
Map<String, ArmState> waypoints = new HashMap<>();

// Axis properties
double heightMeters;
double widthMeters;
double heightRadians;
double widthRadians;
Point center;
ArmState origin;

// 1 meter per 200 pixels, 1 pixel = 1/200 meters
final float METER_TO_PIXEL = 200.0;
final float PIXEL_TO_METER = 1 / METER_TO_PIXEL;
final double PIXEL_TO_RAD = 1 / (1080 / (2 * PI));

final double DEGREES_TO_RAD = PI / 180;
final double INCH_TO_METER = 1 / 39.37;

// Robot
final int TEAM_NUMBER = 1683;
final float ROBOT_WIDTH = 0.762 + 2 * 0.08255; // the width of the robot to be drawn
final float ROBOT_HEIGHT = 0.127;
final float WHEEL_RADIUS = 0.0508 * METER_TO_PIXEL;

// Reef
final float REEF_HEIGHT = 71.870 * (float)INCH_TO_METER;

// Offsets
final float ROBOT_TO_REEF_DISTANCE = 6 * (float)INCH_TO_METER;
final float ROBOT_GROUND_CLEARANCE = 1 * (float)INCH_TO_METER;

// Armlevator
final double MIN_ANGLE = 15.5 * DEGREES_TO_RAD;
final double MAX_ANGLE = 90 * DEGREES_TO_RAD;

final double MIN_EXTENSION = 31.875 * INCH_TO_METER;
final double MAX_EXTENSION = 73.125 * INCH_TO_METER;

final double ANGLE_OFFSET = -MIN_ANGLE;
final double EXTENSION_OFFSET = -MIN_EXTENSION;

Armlevator armlevator = new Armlevator(
  new ArmState(MIN_ANGLE, MIN_EXTENSION),
  new ArmState(MAX_ANGLE, MAX_EXTENSION),
  new ArmState(ANGLE_OFFSET, EXTENSION_OFFSET)
);

final double DIST_FROM_BASE = 0.1540002; // m
final double DIST_FROM_CENTER = ROBOT_WIDTH / 2.0 - 0.2667; // m
Point armRoot = new Point(-DIST_FROM_CENTER, DIST_FROM_BASE);

// Colors
int red = color(255, 0, 0);
int green = color(0, 255, 0);
int blue = color(0, 0, 255);
int teal = color(0, 217, 184);

// Path config
Path path;
ArmState start = fromDegrees(28, 1);
ArmState p1 = fromDegrees(60, 1);
ArmState p2 = fromDegrees(64, 1);
ArmState end = fromDegrees(62, 2);

// Trajectory config
Trajectory traj;
boolean showTraj = false;
long startTime = System.currentTimeMillis();
long prevTime = System.currentTimeMillis();

// Constraints
double maxq1dot = 6.0; // rad/s
double maxldot = 6.0; // meters/s
double maxq1ddot = 5.0; // rad/s/s
double maxlddot = 5.0; // meters/s/s

// Bad Points
List<ArmState> badPoints = new ArrayList<>();

public static float toRadians(float degree) {
  return degree * PI / 180;
}

public ArmState fromDegrees(float theta, float l) {
  return new ArmState(toRadians(theta), l);
}

public static double clamp(double value, double low, double high) {
  return Math.max(low, Math.min(value, high));
}

public ArmState bezierP(ArmState start, ArmState p1, ArmState p2, ArmState end, double t) {
  ArmState first = start.times(Math.pow((1 - t), 3));
  ArmState second = p1.times(3 * Math.pow((1 - t), 2) * t);
  ArmState third = p2.times(3 * (1 - t) * t * t);
  ArmState fourth = end.times(t * t * t);
  return first.plus(second).plus(third).plus(fourth);
}

void setup() {
  fullScreen();
  frameRate(160);

  // axis generation
  heightMeters = displayHeight * PIXEL_TO_METER;
  widthMeters = displayWidth * PIXEL_TO_METER;
  heightRadians = displayHeight * PIXEL_TO_RAD;
  widthRadians = displayWidth * PIXEL_TO_RAD;
  center = new Point(widthMeters / 2, heightMeters / 2);
  origin = new ArmState(widthRadians / 6, heightRadians / 2);

  // path and trajectory initiation
  path = new Path(start, p1, p2, end);
  traj = new Trajectory(maxq1dot, maxldot, maxq1ddot, maxlddot, path);

  // construct UI
  cp5 = new ControlP5(this);

  // Default tab to be renamed to trajectory tab instead
  cp5.getTab("default").setLabel("trajectory");

  // flip switch for traj
  cp5.addToggle("show")
    .setPosition(10, height - 40)
    .setSize(50, 20)
    .setValue(false)
    .setMode(ControlP5.SWITCH);

  // ****** text boxes for changing the constraints ******

  Q1D = cp5.addTextfield("max q1 vel")
    .setPosition(10, height - 80)
    .setSize(100, 20)
    .moveTo("constraints")
    .setValue(maxq1dot + "")
    .setAutoClear(false)
    .setInputFilter(2);

  LD = cp5.addTextfield("max l vel")
    .setPosition(10, height - 40)
    .setSize(100, 20)
    .moveTo("constraints")
    .setValue(maxldot + "")
    .setAutoClear(false)
    .setInputFilter(2);

  Q1DD = cp5.addTextfield("max q1 accel")
    .setPosition(120, height - 80)
    .setSize(100, 20)
    .moveTo("constraints")
    .setValue(maxq1ddot + "")
    .setAutoClear(false)
    .setInputFilter(2);

  LDD = cp5.addTextfield("max l accel")
    .setPosition(120, height - 40)
    .setSize(100, 20)
    .moveTo("constraints")
    .setValue(maxlddot + "")
    .setAutoClear(false)
    .setInputFilter(2);

  // Text boxes for adding/updating a point
  Q1 = cp5.addTextfield("q1")
    .setPosition(10, height - 80)
    .setSize(100, 20)
    .moveTo("Points")
    .setValue(path.start.q1 + "")
    .setAutoClear(false)
    .setInputFilter(3);

  L = cp5.addTextfield("l")
    .setPosition(10, height - 40)
    .setSize(100, 20)
    .moveTo("Points")
    .setValue(path.start.l + "")
    .setAutoClear(false)
    .setInputFilter(3);

  WAYPOINT_NAME = cp5.addTextfield("name")
    .setPosition(120, height - 80)
    .setSize(100, 20)
    .setInputFilter(3)
    .moveTo("Points");

  // Button to add/update a point
  cp5.addBang("update")
    .setPosition(120, height - 40)
    .setSize(100, 20)
    .setLabel("Add/update")
    .align(ControlP5.CENTER, ControlP5.CENTER, ControlP5.CENTER, ControlP5.CENTER)
    .moveTo("Points");

  // ****** dropdown for point a ******
  dropdownA = cp5.addScrollableList("A")
    .setPosition(width - 230, 10)
    .setSize(100, 100)
    .setItemHeight(20)
    .setType(ControlP5.DROPDOWN)
    .setBarHeight(20);

  // ****** dropdown for point b ******
  dropdownB = cp5.addScrollableList("B")
    .setPosition(width - 120, 10)
    .setSize(100, 100)
    .setItemHeight(20)
    .setType(ControlP5.DROPDOWN)
    .setBarHeight(20);

  // file name field
  fileName = cp5.addTextfield("fileName")
    .setPosition(10, height - 80)
    .setSize(100, 20)
    .setLabel("Name")
    .setAutoClear(false)
    .setInputFilter(0);
  
  // button to save to json file
  cp5.addBang("save")
    .setPosition(120, height - 80)
    .setSize(50, 20)
    .align(ControlP5.CENTER, ControlP5.CENTER, ControlP5.CENTER, ControlP5.CENTER);

  // button to load a json file
  cp5.addBang("load")
    .setPosition(180, height - 40)
    .setSize(80, 20)
    .setTriggerEvent(Bang.RELEASE)
    .align(ControlP5.CENTER, ControlP5.CENTER, ControlP5.CENTER, ControlP5.CENTER);

  cp5.addBang("reverse")
    .setPosition(180, height - 80)
    .setSize(80, 20)
    .align(ControlP5.CENTER, ControlP5.CENTER, ControlP5.CENTER, ControlP5.CENTER);

  // button to generate the trajectory from the path
  cp5.addBang("generate")
    .setPosition(70, height - 40)
    .setSize(100, 20)
    .setLabel("Generate Trajectory")
    .align(ControlP5.CENTER, ControlP5.CENTER, ControlP5.CENTER, ControlP5.CENTER);

  //  dropdown to choose start
  // add option to have a clear UI
  cp5.addTab("Clear UI");
  
  // Scan for bad points where the arm should not go
  //for (float q1 = 0.0; q1 <= PI; q1 += .03) {
  //  for (float q2 = -PI; q2 <= PI; q2 += .03) {
  //    var state = new ArmState(q1, q2);
  //    if (state.violatesConstraints()) badPoints.add(state);
  //  }
  //}

  reefImage = loadImage("reefscape-2025-reef.png");
}

class Point {
  PVector vec;
  float x;
  float y;

  public Point(PVector vec) {
    this.vec = vec;
    this.x = vec.x;
    this.y = vec.y;
  }

  public Point(double x, double y) {
    this(new PVector( (float)x, (float)y ));
  }

  public Point plus(Point b) {
    return new Point(PVector.add(vec, b.vec));
  }

  public Point minus(Point b) {
    return new Point(PVector.sub(vec, b.vec));
  }

  public Point times(double k) {
    return new Point(PVector.mult(vec, (float)k ));
  }

  public Point times(Point b) {
    PVector bvec = b.vec;
    return new Point(vec.x * bvec.x, vec.y * bvec.y);
  }

  public Point div(double k) {
    return new Point(PVector.div(vec, (float)k ));
  }

  public Point div(Point b) {
    PVector bvec = b.vec;
    return new Point(vec.x / bvec.x, vec.y / bvec.y);
  }

  public Point asPixel() {
    double pixelX = (x + center.x) / PIXEL_TO_METER;
    double pixelY = (-y + center.y) / PIXEL_TO_METER;

    return new Point(pixelX, pixelY);
  }

  public Point asPoint() {
    double pointX = x * PIXEL_TO_METER - center.x;
    double pointY = -y * PIXEL_TO_METER + center.y;

    return new Point(pointX, pointY);
  }

  public double mag() {
    return vec.mag();
  }

  public void lineTo(Point p) {
    stroke(255);
    Point from = this.asPixel();
    Point to = p.asPixel();
    line(from.x, from.y, to.x, to.y);
  }

  public void show() {
    noStroke();
    fill(255);
    Point center = this.asPixel();
    circle(center.x, center.y, 10.0);
  }

  public void show(float radius) {
    noStroke();
    fill(255);
    Point center = this.asPixel();
    circle(center.x, center.y, radius);
  }
  public void show(float radius, int col) {
    noStroke();
    fill(col);
    Point center = this.asPixel();
    circle(center.x, center.y, radius);
  }

  @Override
    public String toString() {
    return "<" + x + ", " + y + ">";
  }
}

class Armlevator {
  ArmState min;
  ArmState max;
  ArmState offset;
  
  ArmState current = new ArmState(0.0, 0.0);

  public Armlevator(
    ArmState min,
    ArmState max,
    ArmState offset
  ) {
    this.min = min;
    this.max = max;
    this.offset = offset;
  }

  public boolean violatesConstraints() {
    double q1 = current.q1;
    double l = current.l;

    return (q1 < min.q1 || q1 > max.q1)
          || (l < min.l || l > max.l);
  }

  public ArmState constrain(ArmState state) {
    return new ArmState(clamp(state.q1, min.q1, max.q1), clamp(state.l, min.l, max.l));
  }

  public ArmState offset(ArmState state) {
    return state.plus(offset);
  }

  public ArmState unoffset(ArmState state) {
    return state.minus(offset);
  }

  // Show the arm using some geometry (forward kinematics)
  public void show() {
    ArmState constrained = constrain(current);
    double c1 = Math.cos(constrained.q1);
    double s1 = Math.sin(constrained.q1);

    Point endEffector = new Point(constrained.l * c1, constrained.l * s1).plus(armRoot);

    armRoot.lineTo(endEffector);
    endEffector.show(10, red);
    armRoot.show(20, green);
  }

  // Show only end effector of this state
  public void showEndEffectorForState(ArmState state, int col) {
    ArmState constrained = constrain(state);
    double c1 = Math.cos(constrained.q1);
    double s1 = Math.sin(constrained.q1);

    Point endEffector = new Point(constrained.l * c1, constrained.l * s1).plus(armRoot);
    endEffector.show(2, col);
  }
}

// Joint space path
class Path {
  ArmState start;
  ArmState p1; // anchor point 1
  ArmState p2; // anchor point 2
  ArmState end;

  public Path(ArmState start, ArmState p1, ArmState p2, ArmState end) {
    this.start = start;
    this.p1 = p1;
    this.p2 = p2;
    this.end = end;
  }

  /**
   * @param t sample cubic bezier based on this internal parameter from 0 to 1
   */
  public ArmState sample(double t) {
    return bezierP(start, p1, p2, end, t);
  }

  /**
   * @param n the number of samples to collect
   */
  public List<ArmState> collectSamples(int n) {
    double dt = 1.0/n;
    double distance = 0.0;
    List<ArmState> result = new ArrayList<>();
    ArmState prevState = sample(0.0);
    for (double t = 0; t <= 1.0; t += dt) {
      ArmState currState = sample(t);
      var diff = currState.minus(prevState);
      distance += diff.mag();
      currState.s = distance;
      currState.v_theta = diff.getAngle();
      result.add(currState); // move on to the next length
      prevState = currState;
    }
    return result;
  }

  public void showPath() {
    sample(0.0).show(6, green);
    for (double t = .02; t < 1.0; t += .02) {
      ArmState pt = sample(t);
      pt.show(3, color(255, 255, 255, 100));
      armlevator.showEndEffectorForState(pt, color(255, 255, 255));
    }
    sample(1.0).show(6, red);

    ArmState startPx = start.asPixel();
    ArmState p1Px = p1.asPixel();
    ArmState p2Px = p2.asPixel();
    ArmState endPx = end.asPixel();

    noFill();
    stroke(color(255, 255, 255, 80));
    bezier(startPx.q1, startPx.l, p1Px.q1, p1Px.l, p2Px.q1, p2Px.l, endPx.q1, endPx.l);
  }

  public void showHandles() {
    start.lineTo(p1);
    end.lineTo(p2);
    start.show(12, green);
    p1.show(12);
    p2.show(12);
    end.show(12, red);
  }
}

class Trajectory {
  double maxq1dot;
  double maxldot;

  double maxq1ddot;
  double maxlddot;

  double totalTimeSeconds;
  List<ArmState> points;
  Path path;

  public Trajectory(
    double maxq1dot,
    double maxldot,
    double maxq1ddot,
    double maxlddot,
    Path path
  ) {
    this.maxq1dot = maxq1dot;
    this.maxldot = maxldot;
    this.maxq1ddot = maxq1ddot;
    this.maxlddot = maxlddot;
    this.path = path;

    parametrizeTrajectory();
  }

  public ArmState sample(double t) {
    int n = points.size();
    for (int i = 1; i < n; i++) {
      ArmState curr = points.get(i);
      ArmState prev = points.get(i-1);
      if (curr.t == t) return curr;
      if (curr.t > t) {
        // interpolate between prev and current
        ArmState diff = curr.minus(prev);
        ArmState prevV = new ArmState(prev.q1dot, prev.l);
        ArmState vDiff = new ArmState(curr.q1dot, curr.l).minus(prevV);
        double t_err = t - prev.t;
        double dt = curr.t - prev.t;
        double k = t_err / dt;
        ArmState qdot = prevV.plus(vDiff.times(k));
        ArmState q = prev.plus(diff.times(k));
        q.q1dot = qdot.q1dot;
        return q;
      }
    }
    return points.get(n-1);
  }

  public void updateConstraints() {
    maxq1dot = Double.parseDouble(Q1D.getText());
    maxldot = Double.parseDouble(LD.getText());
    maxq1ddot = Double.parseDouble(Q1DD.getText());
    maxlddot = Double.parseDouble(LDD.getText());
  }
  
  // O(n) n = number of points sampled
  public void parametrizeTrajectory() {

    // collect samples from path to use for generating trajectory
    points = path.collectSamples(1000);

    try {
      updateConstraints();
    }
    catch(Exception e) {
      // do nothing
    }

    double maxV = Math.sqrt(maxq1dot * maxq1dot + maxldot * maxldot);
    int n = points.size();
    // ***************************
    // parametrize trajectory here

    // Step 0: set every velocity to the maximum that follows all constraints
    for (ArmState point : points) {
      point.v = maxV;
      point.step = 0;
    }

    // Step 1: Apply curvature constraints,
    // Allows the arm to slow down on tight changes in direction of v
    for (int i = 1; i < n; i++) {
      ArmState currPt = points.get(i);
      ArmState prevPt = points.get(i-1);
      double ds = Math.abs(currPt.s - prevPt.s);
      double dtheta = currPt.v_theta - prevPt.v_theta;
      // wrap around dtheta
      if (dtheta > PI)
        dtheta -= 2 * PI;
      else if (dtheta < -PI)
        dtheta += 2 * PI;
      double curvature = Math.abs(dtheta / ds);
      double sa = Math.abs(Math.sin(currPt.v_theta));
      double ca = Math.abs(Math.cos(currPt.v_theta));
      if ((currPt.v * currPt.v * curvature * sa) > maxq1ddot) {
        currPt.step = 1;
        currPt.v = Math.min(currPt.v, Math.sqrt(maxq1ddot / sa / curvature));
      }
      if ((currPt.v * currPt.v * curvature * ca) > maxlddot) {
        currPt.step = 1;
        currPt.v = Math.min(currPt.v, Math.sqrt(maxlddot / ca / curvature));
      }
    }

    // Step 2: Forward pass, start with v as 0.0
    // Use equation vf = sqrt(v0^2 + 2ad)
    points.get(0).v = 0.0; // start at v = 0
    for (int i = 1; i < n; i++) {
      ArmState currPt = points.get(i);
      ArmState prevPt = points.get(i-1);
      double ds = Math.abs(currPt.s - prevPt.s);
      double ctheta = Math.abs((Math.cos(currPt.v_theta)));
      double stheta = Math.abs((Math.sin(currPt.v_theta)));
      double maxA = (ctheta * maxq1ddot + stheta * maxlddot);
      double vi = prevPt.v;
      if (Math.sqrt(vi * vi + 2.0 * maxA * ds) <= currPt.v) currPt.step = 2;
      double vf = Math.min(currPt.v, Math.sqrt(vi * vi + 2.0 * maxA * ds));
      currPt.v = vf;
    }

    // Step 3: Backward pass
    // Use equation vf = sqrt(v0^2 + 2ad)
    points.get(n-1).v = 0.0; // start at v = 0
    for (int i = n-2; i >= 0; i--) {
      ArmState currPt = points.get(i);
      ArmState prevPt = points.get(i+1);
      double ds = Math.abs(currPt.s - prevPt.s);
      double ctheta = Math.abs((Math.cos(currPt.v_theta)));
      double stheta = Math.abs((Math.sin(currPt.v_theta)));
      double maxA = (ctheta * maxq1ddot + stheta * maxlddot);
      double vi = prevPt.v;
      if (Math.sqrt(vi * vi + 2.0 * maxA * ds) <= currPt.v) currPt.step = 3;
      double vf = Math.min(currPt.v, Math.sqrt(vi * vi + 2.0 * maxA * ds));
      currPt.v = vf;
      currPt.q1dot = (float)(currPt.v * Math.cos(currPt.v_theta));
      currPt.ldot = (float)(currPt.l * Math.sin(currPt.v_theta));
    }
    // ***************************
    // Step 4: find t at each point
    points.get(0).t = 0;
    for (int i = 1; i < points.size(); i++) {
      ArmState curr = points.get(i);
      ArmState prev = points.get(i-1);
      double ds = curr.s - prev.s;
      double dt = 2 * ds / (curr.v + prev.v);
      curr.t = (float)(prev.t + dt);
      totalTimeSeconds = curr.t;
    }
  }

  public void save(String fileName) {
    JSONObject object = new JSONObject();
    
    // gather points into array
    int nStates = points.size();
    JSONArray array = new JSONArray();

    for (int i = 0; i < nStates; i++) {
      JSONObject currState = new JSONObject();
      ArmState state = points.get(i);
      currState.setFloat("t", (float)state.t);
      currState.setFloat("q1", state.q1);
      currState.setFloat("l", state.l);
      currState.setFloat("q1d", state.q1dot);
      currState.setFloat("ld", state.ldot);
      array.setJSONObject(i, currState);
    }
    object.setJSONArray("states", array);

    // save info about this path for reverse engineering
    JSONObject pathObj = new JSONObject();

    pathObj.setInt("nStates", nStates);
    if (nStates > 0) {
      ArmState lastState = points.get(nStates - 1);
      pathObj.setFloat("totalTime", (float)lastState.t);
    } else {
      pathObj.setFloat("totalTime", 0);
    }
    
    ArmState pathStart = armlevator.offset(path.start);
    JSONObject start = new JSONObject();
    start.setFloat("q1", pathStart.q1);
    start.setFloat("l", pathStart.l);
    pathObj.setJSONObject("start", start);

    ArmState pathP1 = armlevator.offset(path.p1);
    JSONObject p1 = new JSONObject();
    p1.setFloat("q1", pathP1.q1);
    p1.setFloat("l", pathP1.l);
    pathObj.setJSONObject("p1", p1);

    ArmState pathP2 = armlevator.offset(path.p2);
    JSONObject p2 = new JSONObject();
    p2.setFloat("q1", pathP2.q1);
    p2.setFloat("l", pathP2.l);
    pathObj.setJSONObject("p2", p2);

    ArmState pathEnd = armlevator.offset(path.end);
    JSONObject end = new JSONObject();
    end.setFloat("q1", pathEnd.q1);
    end.setFloat("l", pathEnd.l);
    pathObj.setJSONObject("end", end);

    object.setJSONObject("path", pathObj);
    
    // save to filename.json
    saveJSONObject(object, fileName + ".json");
  }

  public void show() {
    for (ArmState pt : points) {
      switch(pt.step) {
        case 0:
          pt.show((float)pt.v + .2, color(255));
          break;
        case 1:
          pt.show((float)pt.v + .2, green);
          break;
        case 2:
          pt.show((float)pt.v + .2, blue);
          break;
        case 3:
          pt.show((float)pt.v + .2, red);
          break;
        default:
          pt.show((float)pt.v + .2, color(0, 255, 0));
      }

      armlevator.showEndEffectorForState(pt, color(255));
    }
  }
}

class ArmState {
  float q1;
  float q1dot;
  
  float l;
  float ldot;

  double s; // distance along path
  double v_theta;
  double v;
  float t;
  int step;

  public ArmState(double q1, double l) {
    this.q1 = (float) q1;
    this.l = (float) l;
  }

  public ArmState plus(ArmState b) {
    return new ArmState(q1 + b.q1, l + b.l);
  }

  public ArmState minus(ArmState b) {
    return new ArmState(q1 - b.q1, l - b.l);
  }

  public ArmState times(double k) {
    return new ArmState(q1 * k, l * k);
  }

  public double getAngle() {
    return atan2(l, q1);
  }
  
  public double getY() {
    return l * sin(q1);  
  }
  
  public double getX() {
    return l * cos(q1); 
  }
  
  public ArmState asPixel() {
    double pixelX = (q1 + origin.q1) / PIXEL_TO_RAD;
    double pixelY = (-l + origin.l) / PIXEL_TO_RAD;
    return new ArmState(pixelX, pixelY);
  }

  public ArmState asPoint() {
    double pointX = q1 * PIXEL_TO_RAD - origin.q1;
    double pointY = -l * PIXEL_TO_RAD + origin.l;
    return new ArmState(pointX, pointY);
  }

  // treating point as vector
  public double mag() {
    return sqrt(q1 * q1 + l * l);
  }

  public void lineTo(ArmState p) {
    stroke(255);
    ArmState from = this.asPixel();
    ArmState to = p.asPixel();
    line(from.q1, from.l, to.q1, to.l);
  }

  public void show() {
    noStroke();
    fill(255);
    ArmState center = this.asPixel();
    circle(center.q1, center.l, 10.0);
  }

  public void show(float radius) {
    noStroke();
    fill(255);
    ArmState center = this.asPixel();
    circle(center.q1, center.l, radius);
  }

  public void show(float radius, int col) {
    noStroke();
    fill(col);
    ArmState center = this.asPixel();
    circle(center.q1, center.l, radius);
  }

  @Override
  public String toString() {
    return "(" + q1+ ", " + l + ")";
  }
}

// public void drawRobotAndField() {
//   Point corner1 = new Point(-ROBOT_WIDTH / 2, 0.0).asPixel();
//   stroke(255);
//   fill(red);
//   rect(corner1.x, corner1.y, ROBOT_WIDTH * METER_TO_PIXEL, ROBOT_HEIGHT * METER_TO_PIXEL, 7.0);
//   fill(255);
// }

public void drawRobotAndField() {
  Point robotPosition = new Point(-ROBOT_WIDTH / 2, 0.0);
  Point robotSize = new Point(ROBOT_WIDTH, ROBOT_HEIGHT);

  Point robotPositionPx = robotPosition.asPixel();
  Point robotSizePx = robotSize.times(METER_TO_PIXEL);
  stroke(255);
  fill(red);
  rect(robotPositionPx.x, robotPositionPx.y, robotSizePx.x, robotSizePx.y, 7.0);
  fill(255);

  float reefHeight = REEF_HEIGHT;
  Point robotSizeOffset = robotSize.times(new Point(1, -1));
  // println(robotSizeOffset, new Point(ROBOT_WIDTH, -ROBOT_HEIGHT));
  Point robotToReefOffset = new Point(ROBOT_TO_REEF_DISTANCE, -ROBOT_GROUND_CLEARANCE);
  Point reefPosition = robotPosition.plus(robotSizeOffset).plus(robotToReefOffset).plus(new Point(0, reefHeight)).asPixel();
  
  float reefHeightPx = reefHeight * METER_TO_PIXEL;
  float reefWidthPx = reefHeightPx * ((float)reefImage.width / reefImage.height);

  image(reefImage, reefPosition.x, reefPosition.y, reefWidthPx, reefHeightPx);
}

void draw() {
  long currentTime = System.currentTimeMillis();
  double t = (currentTime - startTime) / 1000.0;
  double dt = (currentTime - prevTime) / 1000.0;

  background(0);
  drawRobotAndField();

  // show the trajectory and path so that it can be modified
  if (showTraj) {
    for (ArmState point : badPoints) {
      point.show(1, red);
    }

    path.showHandles();
    path.showPath();

    traj.show();
  }

  // use velocity instead
  ArmState sample = traj.sample(t % traj.totalTimeSeconds);
  ArmState dv = new ArmState(sample.q1dot, sample.ldot).times(dt);
  armlevator.current = sample.plus(dv);
  prevTime = currentTime;

  armlevator.show();
}

// called by toggle
void show(boolean flag) {
  showTraj = !flag;
}

// add/update the list of waypoints saved
void update() {
  try {
    var q1 = Float.parseFloat(Q1.getText());
    var l = Float.parseFloat(L.getText());
    var name = WAYPOINT_NAME.getText();
    if (name.equals("")) return;
    ArmState pt = fromDegrees(q1, l);
    waypoints.putIfAbsent(name, pt);
    waypoints.replace(name, pt);
    dropdownA.removeItem(name);
    dropdownB.removeItem(name);
    dropdownA.addItem(name, pt);
    dropdownB.addItem(name, pt);
  }
  catch(Exception e) {
    System.err.println("ERROR: Values entered were not numbers.");
  }
}

void A(int n) {
  var item = dropdownA.getItem(n);
  path.start = waypoints.get(item.get("name"));
}

void B(int n) {
  var item = dropdownB.getItem(n);
  path.end = waypoints.get(item.get("name"));
}

void updatePathFromFile(File file) {
  if (file == null) {
    System.err.println("NULL FILE : User did not select a file.");
  } else {
    try {
      // parse the settings of the path
      JSONObject json = loadJSONObject(file);
      JSONObject pathObj = json.getJSONObject("path");

      ArmState pathStart = path.start;
      JSONObject start = pathObj.getJSONObject("start");
      pathStart.q1 = start.getFloat("q1");
      pathStart.l = start.getFloat("l");
      path.start = armlevator.unoffset(pathStart);

      ArmState pathP1 = path.p1;
      JSONObject p1 = pathObj.getJSONObject("p1");
      pathP1.q1 = p1.getFloat("q1");
      pathP1.l = p1.getFloat("l");
      path.p1 = armlevator.unoffset(pathP1);

      ArmState pathP2 = path.p2;
      JSONObject p2 = pathObj.getJSONObject("p2");
      pathP2.q1 = p2.getFloat("q1");
      pathP2.l = p2.getFloat("l");
      path.p2 = armlevator.unoffset(pathP2);

      ArmState pathEnd = path.end;
      JSONObject end = pathObj.getJSONObject("end");
      pathEnd.q1 = end.getFloat("q1");
      pathEnd.l = end.getFloat("l");
      path.end = armlevator.unoffset(pathEnd);

      fileName.setText(file.getName().replace(".json", ""));
    }
    catch(Exception e) {
      // failed to load, dont update current path
      return;
    }
  }
}

void load() {
  selectInput("Select a path to modify", "updatePathFromFile");
}

void save() {
  String file = fileName.getText();
  if (file.contains(".")) return; // trying to save
  traj.save(file);
}

/** method to reverse the path */
void reverse() {
  // swap start and end
  ArmState pointer = path.start;
  path.start = path.end;
  path.end = pointer;
  // swap anchor points
  pointer = path.p1;
  path.p1 = path.p2;
  path.p2 = pointer;
}

void generate() {
  traj.parametrizeTrajectory();
}

//////////////////////////
// Mouse functionality ///
//////////////////////////

boolean draggingA = false;
boolean draggingB = false;
boolean draggingStart = false;
boolean draggingEnd = false;

void mousePressed() {
  if (new ArmState(mouseX, mouseY).minus(path.p1.asPixel()).mag() <= 6.0) {
    draggingA = true;
  } else if (new ArmState(mouseX, mouseY).minus(path.p2.asPixel()).mag() <= 6.0) {
    draggingB = true;
  } else if (new ArmState(mouseX, mouseY).minus(path.start.asPixel()).mag() <= 6.0) {
    draggingStart = true;
  } else if (new ArmState(mouseX, mouseY).minus(path.end.asPixel()).mag() <= 6.0) {
    draggingEnd = true;
  }
}

void mouseDragged() {
  if (draggingA) path.p1 = new ArmState(mouseX, mouseY).asPoint();
  else if (draggingB) path.p2 = new ArmState(mouseX, mouseY).asPoint();
  else if (draggingStart) path.start = new ArmState(mouseX, mouseY).asPoint();
  else if (draggingEnd) path.end = new ArmState(mouseX, mouseY).asPoint();
}

void mouseReleased() {
  draggingA = false;
  draggingB = false;
  draggingStart = false;
  draggingEnd = false;
}
