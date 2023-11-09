import math

flt_alt = 2
center_cord = [320,240]

def get_pixel_cord(cx,cy):

    h = flt_alt
    x1 = cx - center_cord[0]
    y1 = center_cord[1] - cy
    
    print("\nCartesian Coordinates:","x:",x1,"y:",y1,"\n")
    
    theta_rad = math.atan(10.3/109)
    theta_deg = theta_rad*180/math.pi
    print("Theta rad:", theta_rad, "\t\tTheta deg:",theta_deg)
    
    if x1 == 0:
        phi_rad = math.pi/2
    else:
       phi_rad = math.atan(y1/x1)
    
    phi_deg = phi_rad*180/math.pi
    print("Phi rad:", phi_rad, "\t\tPhi deg:",phi_deg)
    
    Rp = math.sqrt(math.pow(x1,2)+math.pow(y1,2)) # Rp is pixel distance
    print("\nRp:",Rp)
    
    Rd = h*math.tan(theta_rad) # Rd is real world distance for 50 pixels
    print("Rd:",Rd)
    
    d = (Rp*Rd)/50.0
    
    x = round(abs(d*math.cos(phi_rad)),5)
    y = round(abs(d*math.sin(phi_rad)),5)
    print("\nX:",x,"Y:",y,"\n")
    
    return (x,y)

get_pixel_cord(320,340)
