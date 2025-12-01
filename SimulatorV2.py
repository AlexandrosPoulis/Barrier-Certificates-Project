import SimulatorV2_classes

#default = input("Whould you like default settings?(Y/N)")
default = "Y"

if default == "N":

    temp_win_size = input("What simulatoion size would you like?height,lenght:\n")
    win_size = temp_win_size.split(",")
    win_size = [int(s) for s in win_size]

    veh_temp = input("What is the starting position of the vehicle?x,y\n")
    start_veh = veh_temp.split(",")
    start_veh = [int(s) for s in start_veh]
    print(start_veh)
    num_obst = int(input("How many obstacles would you like to have?:\n"))

    obst_positions = []
    obst_height = []
    for i in range (int(num_obst)):
        temp = (input("What are the positions of obstacle number " + str(i+1) +"?x,y\n"))
        temp_1 = temp.split(",")
        obst_positions.append([int(s) for s in temp_1])
        obst_height.append(int(input("What athe heigh obstacle number " + str(i+1) +"?\n")))
    
    with open("last settings.txt") as f:
        print(f.readline())
    sim = SimulatorV2_classes.simulator(window=win_size,veh_pos=start_veh,no_obst=num_obst,
                                        obst_pos=obst_positions,obst_h=obst_height)#obst_pos=obst_positions
    sim.run_sim()

elif default == "Y":
    sim = SimulatorV2_classes.simulator(no_obst=5,veh_pos=[350,300],obst_pos=[[400,290],[100,300],[420,50],[410,165],[300,50]],obst_h=[10,10,10,10,10,10,10])
    #sim = SimulatorV2_classes.simulator()
    sim.run_sim()