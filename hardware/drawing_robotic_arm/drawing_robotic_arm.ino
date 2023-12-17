
#include <Servo.h>
#include <ArduinoJson.h>
#include <math.h>
#include <Notecard.h>

#define SERVO_US_MIN 544
#define SERVO_US_MID 1500
#define SERVO_US_MAX 2400
#define PEN_UP SERVO_US_MIN + 150
#define PEN_DOWN SERVO_US_MID - 30

#define SERVO_ANGLE_MIN 0
#define SERVO_ANGLE_MID 90
#define SERVO_ANGLE_MAX 180

#define PHI 0
#define L1 104
#define L2 79
#define L3 88

#define L1_SQ L1 * L1
#define L2_SQ L2 * L2
#define L3_SQ L3 * L3

#define L2_SQ_MINUS_L1_SQ L2_SQ - L1_SQ
#define L1_SQ_MINUS_L2_SQ L1_SQ - L2_SQ

#define STEPS_PER_MM 1

#define WIFI_SSD "SSID"
#define WIFI_PASSWORD "PASSWORD"
#define PRODUCT_ID "PRIDUCT_ID"

#define WIFI_ENABLE 1

typedef struct {
    double x;
    double y;
} point_t;


Servo joints[3], pen;
StaticJsonDocument<51200> doc;

int home_pose[3] = {SERVO_US_MID, SERVO_US_MIN, SERVO_US_MID}; 
int prev_pose[3] = {SERVO_US_MID, SERVO_US_MIN, SERVO_US_MID}; 
int new_pose[3] = {SERVO_US_MID, SERVO_US_MIN, SERVO_US_MID}; 
float joints_angle[3] = {0, 0, 0};
int prev_pen_pose = PEN_UP;
point_t prev_point, next_point;
J *file, *req;

Notecard notecard;


/***************************** Notecard Config ******************************************/
void initConnectivity() {
  J *req;
  #if WIFI_ENABLE
  req = notecard.newRequest("card.wifi");
  JAddStringToObject(req, "ssid", WIFI_SSD);
  JAddStringToObject(req, "password", WIFI_PASSWORD);
  #else
  req = notecard.newRequest("card.wireless");
  #endif
  notecard.sendRequest(req);
}

void initNotehub() {
  J *req = notecard.newRequest("hub.set");
  JAddStringToObject(req, "product", PRODUCT_ID);
  JAddStringToObject(req, "mode", "continuous");
  JAddNumberToObject(req, "inbound", 1);
  JAddNumberToObject(req, "duration", 1);
  JAddBoolToObject(req, "sync", true);
  notecard.sendRequest(req);
}

bool getResponse(J* req, StaticJsonDocument<51200>& doc) {
    J* rsp = notecard.requestAndResponse(req);

    if (rsp == NULL) {
        // Handle the case where no response is received
        Serial.println("No response received.");
        return false;
    }

    // Print the response for debugging purposes
    char* rspString = JPrint(rsp);
    Serial.println("Response JSON:");
    Serial.println(rspString);

    // Parse the JSON response
    DeserializationError error = deserializeJson(doc, rspString);

    // Release memory for the response
    JDelete(rsp);

    if (error != DeserializationError::Ok) {
        // Handle the case where JSON parsing fails
        Serial.print("Failed to parse JSON response: ");
        Serial.println(error.c_str());
        return false;
    }

    // Successfully received and parsed the response
    return true;
}

/*********************************************************************/


/****************************** INVERSE KINEMATICS **********************/
// Estimates servo angles for the given coordinates within the range of the arm
void inverse_kinematics(point_t p, float *joints_angle) {
    float phi_rad = radians(PHI);
    float x = p.x - L3 * cos(phi_rad);
    float y = p.y - L3 * sin(phi_rad);
    float alpha = atan2(y, x);
    float d = sqrt(x * x + y * y);
    float beta = acos((L2_SQ_MINUS_L1_SQ - d * d) / (-2 * d * L1));
    float gamma = acos((d * d - L2_SQ_MINUS_L1_SQ) / (-2 * L2 * L1));
    joints_angle[0] = degrees(alpha + beta);
    joints_angle[1] = 180 - degrees(gamma);
    joints_angle[2] = 180 - (degrees(alpha + beta + gamma) - 180) + PHI - 90;
}

/*************************************************************************/

/*************************** Servo Control *********************************/
// convert the given angle to the microseconds for running servos
int angle_to_us(float angle) {
  return static_cast<int>((angle - SERVO_ANGLE_MIN) * (SERVO_US_MAX - SERVO_US_MIN) / (SERVO_ANGLE_MAX - SERVO_ANGLE_MIN) + SERVO_US_MIN);
}


void estimate_joints_pose(float *joints_angle, int *pose) {
  for(int i = 0; i < 3; i++) {
    pose[i] = angle_to_us(joints_angle[i]);
  }
}

float lerp(float a, float b, float t) {
  return a + t * (b - a);
}


// Drive servo from one point to another smoothly
void drive_servo(int *prev_pos, int *pos) {
    while(prev_pos[0] != pos[0] ||
        prev_pos[1] != pos[1] ||
        prev_pos[2] != pos[2]) {
        for(int i = 2; i >= 0; i--) {
          if(prev_pos[i] > pos[i]){
            prev_pos[i] -= 1;
            
          } else if(prev_pos[i] < pos[i]) {
            prev_pos[i] +=1;
          }
      
          joints[i].writeMicroseconds(prev_pos[i]);
        }   

        delay(2);
    }
}


// Interpolate between 2 points for smooth transition
void interpolate_points(point_t p1, point_t p2) {
  float totalDistance = sqrt(pow(p2.x - p1.x, 2) + pow(p2.y - p1.y, 2));
  if(totalDistance > 1) {
    for (int step = 0; step <= totalDistance * STEPS_PER_MM; step++) {
      float t = step * (1.0 / (totalDistance * STEPS_PER_MM));
      
      point_t interpolated = {lerp(p1.x, p2.x, t), lerp(p1.y, p2.y, t)};
  
      Serial.print(" interpolated : ");
      Serial.print(interpolated.x);
      Serial.print(" ");
      Serial.println(interpolated.y);
      
      if(interpolated.x >= 100 && interpolated.y > 50) {
        inverse_kinematics(interpolated, joints_angle);
        estimate_joints_pose(joints_angle, new_pose);
        drive_servo(prev_pose, new_pose);
      }
  
      delay(30);
    }
  }
}

void move_pen(int pos, int *prev_pos) {
    while(prev_pen_pose != pos) {
        if(prev_pen_pose > pos) {
          prev_pen_pose -= 1;
          
        } else if(prev_pen_pose < pos) {
          prev_pen_pose +=1;
        }
    
        pen.writeMicroseconds(prev_pen_pose);
        delay(1);
    }

    delay(2000);
}

point_t goto_point(point_t point) {
    move_pen(PEN_UP, &prev_pen_pose);
    point = {point.x/10.0 + 100.0, point.y/10.0 + 50.0};
    inverse_kinematics(point, joints_angle);  
    estimate_joints_pose(joints_angle, new_pose);
    drive_servo(prev_pose, new_pose); 
    delay(1000);

    return point;
}

/*********************************************************************/

void execute(StaticJsonDocument<51200>& doc) {
  JsonArray coordinates = doc["body"]["coordinates"];
  point_t prev_point = { -1, -1};
  
  for (JsonArray row : coordinates) {
    if(!(row.size() == 0)) {
      // move arm without drawing
      prev_point = goto_point({(float)row[0][0], (float)row[0][1]});
      
      //start drawing
      move_pen(PEN_DOWN, &prev_pen_pose);
      for (JsonArray coordinate : row) {
        point_t point = {(float)coordinate[0]/10.0 + 100.0, (float)coordinate[1]/10.0 + 50.0};        
        if(prev_point.x >= 0 && prev_point.y >= 0) {
          interpolate_points(prev_point, point);
          delay(1000);
        } 
        
        prev_point.x = point.x;
        prev_point.y = point.y;
      }    

      inverse_kinematics(prev_point, joints_angle);  
      estimate_joints_pose(joints_angle, new_pose);
      drive_servo(prev_pose, new_pose); 
      delay(1000);
    }
  }

  move_pen(PEN_UP, &prev_pen_pose);
  drive_servo(prev_pose, home_pose);
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(9600);

  //Init notecard
  notecard.begin();
  initConnectivity();
  initNotehub();
  delay(10);

  // Init Servos
  joints[0].attach(5);
  joints[1].attach(9);
  joints[2].attach(12);
  pen.attach(13);
  delay(10);

  // Move Servos to the default position
  joints[0].writeMicroseconds(SERVO_US_MID);
  joints[1].writeMicroseconds(SERVO_US_MIN);
  joints[2].writeMicroseconds(SERVO_US_MID);
  pen.writeMicroseconds(PEN_UP);
  delay(3000);

  digitalWrite(LED_BUILTIN, HIGH);
}

void loop() {
  // Check for data.qi file updates
  req = notecard.newRequest("file.changes");
  file = JAddArrayToObject(req, "files");
  JAddItemToArray(file, JCreateString("data.qi"));

  // if new data is available then parse it and execute drawing
  if(getResponse(req, doc) && doc["info"]["data.qi"]["total"] > 0){
    req = notecard.newRequest("note.get");
    JAddStringToObject(req, "file", "data.qi");
    JAddBoolToObject(req, "delete", true);
    
    if(getResponse(req, doc)) {
      execute(doc);
    }
  }

  // Check update every 5 seconds
  delay(5000);
}
