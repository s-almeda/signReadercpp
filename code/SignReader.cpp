/******************************************************************************\
* Copyright (C) 2012-2014 Leap Motion, Inc. All rights reserved.               *
* Leap Motion proprietary and confidential. Not for distribution.              *
* Use subject to the terms of the Leap Motion SDK Agreement available at       *
* https://developer.leapmotion.com/sdk_agreement, or another agreement         *
* between Leap Motion and you, your company or other organization.             *
\******************************************************************************/

#define _USE_MATH_DEFINES
#include <iostream>
#include <cstring>
#include "Leap.h"
#include <array>
#include <map>
#include <list>
#include <math.h>
#include <stdlib.h> 
#include <stdio.h> 

using namespace Leap;


int EXTENDED_MARGIN = 9;
int CURLED_MARGIN = 10;
int DOWN_MARGIN = 0;
int THUMB_IN_MARGIN = 10;
int THUMB_UP_MARGIN = 20; //#if angle between thumb and hand is < this, thumb is up
int MAX_FRAMES_HISTORY = 30; //max number of frames to store 
int MAX_EQUAL_FINGER_DISTANCE = 2; // if fingers in 2 consecutive frames exceed this distance, the hand is not steady.
int SIDEWAYS_ROLL_DIFFERENCE = 25; // if the difference between the roll and 90 is < this, the palm is sideways
int TOUCHING_MARGIN = 30;


class MyListener : public Listener {
  public:
    virtual ~MyListener() {};
    virtual void onInit(const Controller&);
    virtual void onConnect(const Controller&);
    virtual void onDisconnect(const Controller&);
    virtual void onExit(const Controller&);
    virtual void onFrame(const Controller&);
    virtual void onFocusGained(const Controller&);
    virtual void onFocusLost(const Controller&);
    virtual void onDeviceChange(const Controller&);
    virtual void onServiceConnect(const Controller&);
    virtual void onServiceDisconnect(const Controller&);
  

  private:
};

const std::string fingerNames[] = {"Thumb", "Index", "Middle", "Ring", "Pinky"};
const std::string boneNames[] = {"Metacarpal", "Proximal", "Middle", "Distal"};
const std::string stateNames[] = {"STATE_INVALID", "STATE_START", "STATE_UPDATE", "STATE_END"};



void MyListener::onInit(const Controller& controller) {
  std::cout << "Initialized" << std::endl;
}

void MyListener::onConnect(const Controller& controller) {
  std::cout << "Connected" << std::endl;
  controller.enableGesture(Gesture::TYPE_CIRCLE);
  controller.enableGesture(Gesture::TYPE_KEY_TAP);
  controller.enableGesture(Gesture::TYPE_SCREEN_TAP);
  controller.enableGesture(Gesture::TYPE_SWIPE);
}

void MyListener::onDisconnect(const Controller& controller) {
  // Note: not dispatched when running in a debugger.
  std::cout << "Disconnected" << std::endl;
}

void MyListener::onExit(const Controller& controller) {
  std::cout << "Exited" << std::endl;
}

void MyListener::onFrame(const Controller& controller) {
  std::list<Frame> frames;
  // Get the most recent frame and report some basic information
  Frame frame = controller.frame();
  if (frames.size() >= MAX_FRAMES_HISTORY){
    frames.pop_front();
    frames.push_back(frame);
  }
  else{
    frames.push_back(frame);
  }
  

}

void MyListener::onFocusGained(const Controller& controller) {
  std::cout << "Focus Gained" << std::endl;
}

void MyListener::onFocusLost(const Controller& controller) {
  std::cout << "Focus Lost" << std::endl;
}

void MyListener::onDeviceChange(const Controller& controller) {
  std::cout << "Device Changed" << std::endl;
  const DeviceList devices = controller.devices();

  for (int i = 0; i < devices.count(); ++i) {
    std::cout << "id: " << devices[i].toString() << std::endl;
    std::cout << "  isStreaming: " << (devices[i].isStreaming() ? "true" : "false") << std::endl;
  }
}

void MyListener::onServiceConnect(const Controller& controller) {
  std::cout << "Connected!" << std::endl;
}

void MyListener::onServiceDisconnect(const Controller& controller) {
  std::cout << "Disconnected. :,-(" << std::endl;
}




void printData(const Controller& controller){
  const Frame frame = controller.frame();
    std::cout << "Frame id: " << frame.id()
              << ", timestamp: " << frame.timestamp()
              << ", hands: " << frame.hands().count()
              << ", extended fingers: " << frame.fingers().extended().count()
              << ", tools: " << frame.tools().count()
              << ", gestures: " << frame.gestures().count() << std::endl;

    HandList hands = frame.hands();
    for (HandList::const_iterator hl = hands.begin(); hl != hands.end(); ++hl) {
      // Get the first hand
      const Hand hand = *hl;
      std::string handType = hand.isLeft() ? "Left hand" : "Right hand";
      std::cout << std::string(2, ' ') << handType << ", id: " << hand.id()
                << ", palm position: " << hand.palmPosition() << std::endl;
      // Get the hand's normal vector and direction
      const Vector normal = hand.palmNormal();
      const Vector direction = hand.direction();

      // Calculate the hand's pitch, roll, and yaw angles
      std::cout << std::string(2, ' ') <<  "pitch: " << direction.pitch() * RAD_TO_DEG << " degrees, "
                << "roll: " << normal.roll() * RAD_TO_DEG << " degrees, "
                << "yaw: " << direction.yaw() * RAD_TO_DEG << " degrees" << std::endl;

      // Get the Arm bone
      Arm arm = hand.arm();
      std::cout << std::string(2, ' ') <<  "Arm direction: " << arm.direction()
                << " wrist position: " << arm.wristPosition()
                << " elbow position: " << arm.elbowPosition() << std::endl;

      // Get fingers
      const FingerList fingers = hand.fingers();
      for (FingerList::const_iterator fl = fingers.begin(); fl != fingers.end(); ++fl) {
        const Finger finger = *fl;
        std::cout << std::string(4, ' ') <<  fingerNames[finger.type()]
                  << " finger, id: " << finger.id()
                  << ", length: " << finger.length()
                  << "mm, width: " << finger.width() << std::endl;

        // Get finger bones
        for (int b = 0; b < 4; ++b) {
          Bone::Type boneType = static_cast<Bone::Type>(b);
          Bone bone = finger.bone(boneType);
          std::cout << std::string(6, ' ') <<  boneNames[boneType]
                    << " bone, start: " << bone.prevJoint()
                    << ", end: " << bone.nextJoint()
                    << ", direction: " << bone.direction() << std::endl;
        }
      }
  }

  // Get tools
  const ToolList tools = frame.tools();
  for (ToolList::const_iterator tl = tools.begin(); tl != tools.end(); ++tl) {
    const Tool tool = *tl;
    std::cout << std::string(2, ' ') <<  "Tool, id: " << tool.id()
              << ", position: " << tool.tipPosition()
              << ", direction: " << tool.direction() << std::endl;
  }

  // Get gestures
  const GestureList gestures = frame.gestures();
  for (int g = 0; g < gestures.count(); ++g) {
    Gesture gesture = gestures[g];

    switch (gesture.type()) {
      case Gesture::TYPE_CIRCLE:
      {
        CircleGesture circle = gesture;
        std::string clockwiseness;

        if (circle.pointable().direction().angleTo(circle.normal()) <= PI/2) {
          clockwiseness = "clockwise";
        } else {
          clockwiseness = "counterclockwise";
        }
      }
      case Gesture::TYPE_SWIPE:
      {
        SwipeGesture swipe = gesture;
        std::cout << std::string(2, ' ')
          << "Swipe id: " << gesture.id()
          << ", state: " << stateNames[gesture.state()]
          << ", direction: " << swipe.direction()
          << ", speed: " << swipe.speed() << std::endl;
        break;
      }
      case Gesture::TYPE_KEY_TAP:
      {
        KeyTapGesture tap = gesture;
        std::cout << std::string(2, ' ')
          << "Key Tap id: " << gesture.id()
          << ", state: " << stateNames[gesture.state()]
          << ", position: " << tap.position()
          << ", direction: " << tap.direction()<< std::endl;
        break;
      }
      case Gesture::TYPE_SCREEN_TAP:
      {
        ScreenTapGesture screentap = gesture;
        std::cout << std::string(2, ' ')
          << "Screen Tap id: " << gesture.id()
          << ", state: " << stateNames[gesture.state()]
          << ", position: " << screentap.position()
          << ", direction: " << screentap.direction()<< std::endl;
        break;
      }
      default:
        std::cout << std::string(2, ' ')  << "Unknown gesture type." << std::endl;
        break;
    }
  }

  if (!frame.hands().isEmpty() || !gestures.isEmpty()) {
    std::cout << std::endl;
  }
}


FingerList getFingers(Frame frame, bool printRequested){
  FingerList fingers;
  //const char finger_names[5] = {'T', 'I', 'M', 'R', 'P'};
    if (!frame.hands().isEmpty()){ 
        HandList hands = frame.hands();
        for (HandList::const_iterator hl = hands.begin(); hl != hands.end(); ++hl) {
        // Get the first hand
        Hand hand = *hl;
        if (printRequested)
          std::string handType = hand.isLeft() ? "Left hand" : "Right hand";
        // Get fingers
        fingers = hand.fingers();
        for (FingerList::const_iterator fl = fingers.begin(); fl != fingers.end(); ++fl) {
          const Finger finger = *fl;
          if (printRequested){
            std::cout << std::string(4, ' ') <<  fingerNames[finger.type()]
                    << " finger, id: " << finger.id()
                    << ", length: " << finger.length()
                    << "mm, width: " << finger.width() << std::endl;
          }
        } //end of fingers
      } //end of hands
    }

    return fingers;

}

 
bool framesEqual(Frame frame1, Frame frame2){
  /* """framesEqual() function returns true if the 2 frames are about equal

        takes 2 frames and checks each finger (each frame should only have 1 hand)

        Parameters
        ----------
        frame1: frame
            first frame
        frame2: frame
            second frame to compare

        Returns
        -------
        bool
            true if frames are about equal, false if not
        """*/

  
    if (frame1.hands().isEmpty()|| frame2.hands().isEmpty())
        return false;
    

    for (int i = 0; i <= 5; i++){ //iterates from 0(TYPE_THUMB) to 4 (TYPE_PINKY):
        FingerList group1 = getFingers(frame1, false);
        FingerList group2 = getFingers(frame2, false);

        if (group1.count() != group2.count()) //if one frame has more fingers than the other frame
            return false;

        


        float distance;
        for (int i = 0; i < group1.count(); i++){
            distance = (group1[i].stabilizedTipPosition()).distanceTo(group2[i].stabilizedTipPosition());
            //std::cout << "DISTANCE BETWEEN FINGER " << i << ": " << distance << std::endl;
            if (distance >= MAX_EQUAL_FINGER_DISTANCE)
                return false;
        }
    }

    return true;
}

/*****
  """ isSteady function returns true if the hand is steady
      uses the framesEqual(frame1, frame2) function

            checks if the last (framecount) frames in the list (frameList) are about equal
            if so, the hand is steady, return true. otherwise return false.

            Parameters
            ----------
            frameList : list of frames
                list of recent frames, should have more than framecount
            frameCount : int
                number of frames to check

            Returns
            -------
            bool
                true if hand is steady, false otherwise.
***/
/*bool isSteady (list<Frame> frameList, int frameCount){
   int total = frameList.size();

    Frame frame1 = frameList[total - frameCount] //first frame that we want to check
    int i = (total - frameCount + 1)           //index of the frame after that one
    Frame frame2 = frameList[i]

    for (int x = i+1; x < frameCount; x++){       //iterate through the selected frames

        if (not(framesEqual(frame1, frame2))): // if they're not about equal,
           std::cout << "NOT STEADY";
            return false;                    #then quit
        frame1 = frame2;                    #otherwise, iterate to the next 2 frames.
        frame2 = frameList[x];

      }
    std::cout <<"STEADY";
    return true;           

}*/

bool isSteady(const Controller& controller){
    if (framesEqual(controller.frame(1), controller.frame()) && framesEqual(controller.frame(1), controller.frame(2)))
        return true;
    return false;
}

bool isSteady(const Controller& controller, int frameCount){
  Frame frame1 = controller.frame();
  Frame frame2 =controller.frame(1);
    for (int i = 1; i < frameCount; i++){
      if ( (!(frame1.isValid())) || (!(frame2.isValid())) || (!(framesEqual(frame1, frame2))))
        return false; //if the frames are not equal or one of them is invalid
      frame1 = frame2;
      frame2 = controller.frame(i);
    }
    return true;
}

std::vector<Bone::Type> getBoneTypes(){
  std::vector<Bone::Type> result;
  Bone::Type METACARPAL = static_cast<Bone::Type>(0);
  result.push_back(METACARPAL);
  Bone::Type PROXIMAL = static_cast<Bone::Type>(1);
  result.push_back(PROXIMAL);
  Bone::Type INTERMEDIATE = static_cast<Bone::Type>(2);
  result.push_back(INTERMEDIATE);
  Bone::Type DISTAL = static_cast<Bone::Type>(3);
  result.push_back(DISTAL);

  return result;

}

/*#######################################################################
#
# getFingerStatuses - used to get status of each finger                 #
#   either extended, or bent, and a bent finger can be curled or down   #
#
########################################################################*/

std::map<std::string, std::string> getFingerStatuses(FingerList fingers, Hand hand, bool printRequested){
    
    std::string fingerNames[] = {"Thumb", "Index", "Middle", "Ring", "Pinky"};
    std::map<std::string, std::string> result;
    std::string status = "normal";
    //fingerAngles = dict(getFingerAngles(fingers, hand, False))
    //angleToPalm = getAnglesToPalm(fingers, hand, False);
    //angleToHand = getAnglesToHand(fingers, hand, False);

    

    std::vector<Bone::Type> bones = getBoneTypes();
    

   // #GET PALM STATUS
    float roll = (180*(hand.direction().roll())/M_PI);
    if (abs(abs(roll) - 90) < SIDEWAYS_ROLL_DIFFERENCE)
        status = "SIDEWAYS";
    if (abs(roll) < SIDEWAYS_ROLL_DIFFERENCE)
        status = "DOWN";
    result.insert(std::make_pair("palm", status));


    status = "unknown";
    for (Finger f : fingers){
        //GET DATA

        float shortLength = f.length();
        float metacarpalLength = (f.bone(bones[0]).length()/2); //length of half the metacarpal
        float fullLength = shortLength + metacarpalLength; //estimated length from center of palm to tip of finger
        Vector palmCenter = hand.palmPosition();
        float tipToPalm = abs(f.stabilizedTipPosition().distanceTo(palmCenter) - f.length()); //distance from finger tip to palm
        float nonAbsTipToPalm = f.stabilizedTipPosition().distanceTo(palmCenter) - f.length();
        //float tipToKnucks = (f.stabilizedTipPosition().distanceTo(f.bone(bones[0]).nextJoint()))- f.length(); // distance from finger tip to knuckles
        float tipToPalmA = f.bone(bones[3]).nextJoint().distanceTo(palmCenter); //distance from finger tip to palm
        
        //--DECIDE FINGER STATUS
        //--special logic for thumb
        if (f.type() == 0){
            float thumbPalmAngle = (180*((f.direction().angleTo(hand.palmNormal())))/M_PI);
            float thumbHandAngle = (180*((f.direction().angleTo(hand.direction())))/M_PI);
            if (printRequested)
                std::cout << "thumbHandAngle : " << thumbHandAngle << std::endl;
            if (thumbHandAngle < THUMB_UP_MARGIN)
                status = "up"; //the thumb is up
                
            else{
                status = "bent";
                if ((tipToPalm -shortLength) < EXTENDED_MARGIN || abs(thumbPalmAngle- 90) < EXTENDED_MARGIN)
                        status = "out";
                if ((tipToPalm < THUMB_IN_MARGIN) && thumbHandAngle > 20) //if middle of thumb is closer to palm than base of thumb
                        status = "in";

            }
        }
        //for all fingers except the thumb
        else{
          float angleToPalm = (180*((f.direction().angleTo(hand.palmNormal())))/M_PI);
          float angleToHand = (180*((f.direction().angleTo(hand.direction())))/M_PI);
          float intToPalm = f.bone(bones[2]).center().distanceTo(palmCenter); //distance from finger center to palm
            if (angleToPalm > angleToHand && (tipToPalmA - fullLength) > EXTENDED_MARGIN)
                status = "up"; //the finger is extended

            else{
                std::cout << "tip to palm - short length " <<  (tipToPalm - shortLength) << std::endl;
                status = "bent";
                
              
                if ((tipToPalmA - intToPalm) > CURLED_MARGIN && nonAbsTipToPalm > CURLED_MARGIN)
                    status = "curled";

                if ((tipToPalmA - shortLength) < DOWN_MARGIN)
                    status = "down";
                


                else{                        
                //if the finger to palm angle is small and the finger to hand direction is large, then the finger is probably pointing straight out.
                    if (angleToHand > 50 && angleToHand < 100)
                        status = "out";
                }

            }
        }

                
       result.insert(std::make_pair(fingerNames[f.type()], status));
    }

    if (printRequested){
      std::map<std::string, std::string>::iterator pos;
      for (pos = result.begin(); pos != result.end(); ++pos) {
          std::cout << pos->first << " Finger Status: " << pos->second << std::endl;
        }
    }


    return result;
}

bool isTouching (FingerList fingers, Finger::Type finger1, Finger::Type finger2, bool printRequested){

  /*"""
    returns whether the two finger tips are touching
    that is, the distance is within the isTouchingMargin

    Parameters
    ----------
    

    Returns
    -------
        True if points are in touching distance,
        False if not.
    """*/

    FingerList fingers1 = fingers.fingerType(finger1);
    FingerList fingers2 = fingers.fingerType(finger2);
    if (fingers1.isEmpty() || fingers2.isEmpty()){
      return false;
    }
    
    std::vector<Bone::Type> bones = getBoneTypes();
    Vector boneCoord1 = fingers1[0].bone(bones[3]).nextJoint();
    Vector boneCoord2 = fingers2[0].bone(bones[3]).nextJoint();

    float thisDistance = boneCoord1.distanceTo(boneCoord2);
    if (printRequested)
      std::cout << "Distance: " << thisDistance;

    if (thisDistance < TOUCHING_MARGIN){
        if (printRequested)
            std::cout<< "touching";
        return true;
    }
    if(printRequested)
        std::cout<< "not touching";
    return false;

  

}



int main(int argc, char** argv) {
  /* Main Function, body of program */

  //bool paused = false;
  //bool recording = false;


  // Create a sample listener and controller
  MyListener listener;
  Controller controller;


  //std::list<Frames>* = listener.frames;

 

  // Have the sample listener receive events from the controller
  controller.addListener(listener);

  if (argc > 1 && strcmp(argv[1], "--bg") == 0)
    controller.setPolicy(Leap::Controller::POLICY_BACKGROUND_FRAMES);


  std::string input;
  FingerList myFingerList;
  Hand myHand;

  Frame mostRecentFrame = controller.frame();


  // Keep this process running until Enter is pressed
  std::cout << "Enter q to quit..." << std::endl;
  //getline(std::cin, input);
  while(!(input[0] == 'q')){
    
    getline(std::cin, input); //GET INPUT

    /*
    # 1     -> print ALL INFO
    # 2     -> print FINGER INFO
    # 3     -> print FINGER STATUSES
    # s     -> pause recording
    # ' '   -> get sign (spacebar)

    */
    if (!(input.empty())){ //EMPTY INPUT
      //std::cout << "inputted:" << input << std::endl;
    }
    if (input[0] == '1'){ // print ALL INFO
        printData(controller);
    }
    if (input[0] == '2'){ // print FINGER INFO
        getFingers(controller.frame(), true);
    }

    if (input[0] == '3'){ 
      mostRecentFrame = controller.frame();
      if (isSteady(listener)){
                    myFingerList = getFingers(mostRecentFrame, false);
                    myHand = mostRecentFrame.hands().frontmost();
                    std::map<std::string, std::string> fingerStatuses = getFingerStatuses(myFingerList, myHand, true);
      } 
      else{
        std::cout<< "Hand is unsteady.";
      }

    }


    if (input[0] == 's'){ // print if steady
        if (isSteady(controller)){
            std::cout<< "STEADY\n";
        }
        else{
          std::cout<< "NOT STEADY\n";
        }
    }

    /**** #~ GET SIGN ~#  *****/
    if (input[0] == ' '){ 
      mostRecentFrame = controller.frame();
      if (isSteady(listener)){
                    myFingerList = getFingers(mostRecentFrame, false);
                    myHand = mostRecentFrame.hands().frontmost();
                    std::map<std::string, std::string> fingerStatuses = getFingerStatuses(myFingerList, myHand, true);

                    //getSign(fingerStatuses, myHand, isTouching(myFingerList, 0, 2, false));
      } 
                    //anglesToPalm(myFingerList, myHand, False), getFingerAngles(myFingerList,myHand, False), 
      else{
        std::cout<< "Hand is unsteady.";
      }

    }
  }





  // Remove the sample listener when done
  controller.removeListener(listener);

  return 0;
}
