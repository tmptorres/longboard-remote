
float map2Float(long x, long in_min, long in_max, float out_min, float out_max){
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float currentSlewRateFilter(float x, float xDelayed, float maxSlewRate, float minSlewRate){

  // maxSlewRate in A/Sample
  // minSlewRate in A/Sample

  // If current returns to zero, inconditionally set it to zero (aka release the motor)
  if(abs(x) < 0.1){
    return 0;
  }
  
  float topLim = xDelayed + maxSlewRate;
  float bottomLim = xDelayed + minSlewRate;
  
  if(x > topLim){
      x = topLim;
  }
  if(x < bottomLim){
      x = bottomLim;
  }
  return x;
}

int pulseSlewRateFilter(int x, int xDelayed, int maxSlewRate, int minSlewRate){

  // maxSlewRate in A/Sample
  // minSlewRate in A/Sample
  
  int topLim = xDelayed + maxSlewRate;
  int bottomLim = xDelayed + minSlewRate;
  
  if(x > topLim){
      x = topLim;
  }
  if(x < bottomLim){
      x = bottomLim;
  }
  return x;
}
