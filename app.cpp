void App:checkPosition(){
  if (position.y > 490){
    position.y = 490;
  }
  else if (position.y < 10){
    position.y = 10;
  }
  if (position.z > 590){
    position.z = 590;
  }
  else if (position.z < -300){
    position.z = -300;
  }
  if (position.x > 480){
    position.x = 480;
  }
  else if (position.x < -200){
    position.x = -200;
  }
  else if (position.x >110 && position.x < 480){
    if (position.y < 140 && position.z > 250){
      position.y = 140;
      postion.z = 250;
    }
    else if (((position.z > -20) && position.z <250) && (position.y < (position.z + 20) * 14.0 / 27)) {
      position.y = (position.z + 20) * 14.0 / 27);
    }
  }
}
