double feedback_pitch(){

    static double m = 1.292; //dalam Kg
    static double g = 9.80665; //dalam m/s^2
    static double l = 0.1638887897; // dalam m
    static double ixx = 0.04076112625; 
    static double K_pitch = 9.8888; // nilai K yg didapat dari hasil penalaan LQR 
    static double K_dot_pitch = 0.8494; // nilai K yg didapat dari hasil penalaan LQR
    static double ankle_pitch,pitch_prev;
    double ref_pitch,pitch_now,error_pitch,error_pitch_dt;
    double u_Pitch, alpha_pitch,pos_pitch,vel_pitch;
    double Ts = 0.02; //waktu sampling
    
    ref_pitch = 0; 
    pitch_now = deg2rad(imu_pitch);//pembacaan sudut pitch saat ini yang didapat dari imu
    std::cout<< "rad pitch : " << imu_pitch << std::endl;
    error_pitch = pitch_now - ref_pitch; //error pembacaan imu
    error_pitch_dt = error_pitch/Ts; 
    std::cout<< "error pitch : " << error_pitch << std::endl;
    
    K_pitch = K_pitch;
    u_Pitch = ((-K_pitch*(error_pitch-ref_pitch))+(-K_dot_pitch*error_pitch_dt)); //fullstate feedback
    alpha_pitch = ((u_Pitch+(m*g*l*sin(error_pitch)))/ixx); //alpha pendulum
    alpha_pitch = rad2deg(alpha_pitch);
    std::cout<< "alpha pitch : " << alpha_pitch << std::endl;
    pos_pitch = (error_pitch_dt*Ts + 0.5*alpha_pitch*Ts*Ts); //posisi GMBB
    vel_pitch = (error_pitch_dt+alpha_pitch*Ts); //kecepatan GLBB
    ankle_pitch = deg2reg(pos_pitch);
    ankle_pitch = ankle_pitch;
    std::cout<< "ankle_pitch : " << ankle_pitch << std::endl;
    pitch_prev = pitch_now;
    // f = fuzzy(error, error_dt);
    return ankle_pitch; 
}
double feedback_roll(){

    static double m = 1.292; //dalam Kg
    static double g = 9.80665; //dalam m/s^2
    static double l = 0.1638887897; // dalam m
    static double iyy = 0.039692090288333336; 
    static double K_roll = 30.8888; // nilai K yg didapat dari hasil penalaan LQR 
    static double K_dot_roll = 0.8494; // nilai K yg didapat dari hasil penalaan LQR
    static double ankle_roll,roll_prev;
    double ref_roll,roll_now,error_roll,error_roll_dt;
    double u_roll, alpha_roll,pos_roll,vel_roll;
    double Ts = 0.02; //waktu sampling
    
    ref_roll = 0; 
    roll_now = deg2rad(imu_roll);//pembacaan sudut pitch saat ini yang didapat dari imu
    std::cout<< "rad pitch : " << imu_roll << std::endl;
    error_roll = roll_now - ref_roll; //error pembacaan imu
    error_roll_dt = error_roll/Ts; 
    std::cout<< "error roll : " << error_roll << std::endl;
    
    K_roll = K_roll;
    u_roll = ((-K_roll*(error_roll-ref_roll))+(-K_dot_roll*error_roll_dt)); //fullstate feedback
    alpha_roll = ((u_roll+(m*g*l*sin(error_roll)))/iyy); //alpha pendulum
    alpha_roll = rad2deg(alpha_roll);
    std::cout<< "alpha roll : " << alpha_roll << std::endl;
    pos_roll = (error_roll_dt*Ts + 0.5*alpha_roll*Ts*Ts); //posisi GLBB
    vel_roll = (error_roll_dt+alpha_roll*Ts); //kecepatan GLBB
    ankle_roll = deg2reg(pos_roll);
    ankle_roll = ankle_roll;
    std::cout<< "ankle_roll : " << ankle_roll << std::endl;
    roll_prev = roll_now;
   
    return ankle_roll; 
}