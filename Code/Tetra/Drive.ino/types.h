typedef struct  {
  int Rotator;
  int Lift;
  int Kick;
} LegPosition;

typedef struct  {
  LegPosition LeftFront;
  LegPosition RightFront;
  LegPosition LeftBack;
  LegPosition RightBack;
  int Neck;
} RobotState;

typedef struct {
  RobotState frames [16];
  int lengths [16];
  int number_of_frames;
  bool is_looping;
}Animation;

typedef struct{
  int notes [16];
  int lengths [16];
  int number_of_notes;
} Melody;
