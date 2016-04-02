--------------------------------------------------------------------------------------------
    PREPARATION
--------------------------------------------------------------------------------------------

  [0:00]

  # Make sure the RoboEarth router is correctly positioned
  - Disconnect the big switch on the table to make sure that no computers are consuming bandwidth
  - Max and Janno put AMIGO and SERGIO side-by-side in the furthest living room corner
    near the hall, 1.5 meter apart in Y (SERGIO nearer to the bar)
  # AMIGO has torso down, looking down, lights off # ToDo: make direct function
  - Rein has computer with visualization connected to beamer:
    - One desktop with OpenCV GUI
    - One desktop with our custom visualization
  - Rokus and Luis start re-positioning the objects in the kitchen
    - ALSO already put grasp items on the table. # ToDo: which object, where?
  - Ramon sits with the jury with the GUI on the iPad

  - While setting up, Sjoerd starts pitch

  - Luis goes to the bedroom, and stays there

--------------------------------------------------------------------------------------------
    SCENARIO
--------------------------------------------------------------------------------------------

  [0:30] SERGIO stores AMIGO's initial pose in the world model

  [0:30] Using the GUI Ramon maps the room with SERGIO in the order:
         - bar
         - dinner_table
         - ... other objects in livingroom ...
         - bed
         - ... continue to hallway

  [?:??] As soon as SERGIO maps the bar, AMIGO is requested to go to the person nearest to the bar.
         AMIGO's queries his initial pose from the world model and sets it.

         TODO:
             how do we give this command? With the GUI?

         SERGIO keeps on exploring the room

  [?:??] AMIGO asks: "what can I do for you?".
         OPERATOR: "Luis needs your help"
         AMIGO: "Where can I find Luis?"
         OPERATOR: "near the bed"

  [?:??] AMIGO drives to the person near the bed (Luis). He asks Luis: "What can I do for you?"
         Luis: "Bring me the object on the dinner_table"

  [?:??] AMIGO drives to the dinner_table, grasps the object, and bring it back Luis (again the person nearest to the bed)

  [?:??] AMIGO asks Luis: "what can I do for you?".
         Luis: "Go back to the operator"

  [?:??] AMIGO drives back to the person nearest to the bar (Sjoerd)
         AMIGO asks Sjoerd: "what can I do for you?".
         Sjoerd: "How much time do we still have?"

         Rokus' time node is queried. This returns a string that can be directly given to 'speak'
       
  [?:??] From this point on, let AMIGO keep on asking "What can I do for you?". The possibilities are:

         "Get me the object from X"

  [?:??] When SERGIO is done exploring, Ramon clicks a button on the GUI and SERGIO drives back to the jury

         TODO: maybe it's nice to have the option "drive to X" in the GUI. THIS IS A NICE TO HAVE!


--------------------------------------------------------------------------------------------
    TODO PER PERSON
--------------------------------------------------------------------------------------------

  - Ramon: get bandwith check tool to work, responsible for GUI

  - Max: testing
  #    - Make sure we have external audio on SERGIO (ask Janno)
      - Check if we can have a mic for Sjoerd and also one to point at each Robot so that the jury can listen to the orders and responses 
          (3 mics, or 2 if Sergio does not communicate much)

  - Rokus: 
      - time query node 
      - testing

  - Luis: testing

  - Erik: AMIGO executive
      - When started, AMIGO automatically goes down, head down, (lights off)
      - Can be triggered by topic (received from GUI)

  - Janno: SERGIO executive
  #    - SERGIO should always keep it's head straight, slightly looking down
  o    - Must be able to cancel nav goals -> need snappy behavior

  - Rein:
      - robot-robot communication
      - Make sure laptop visualization works with beamer

  - Sjoerd: visualization
  #   - Navigation
      - Localization AMIGO
      - OpenCV GUI mimic
      - RViz-like visualization

--------------------------------------------------------------------------------------------
    NICE TO HAVES (DON'T FOCUS ON THIS, UNLESS ALL OF THE ABOVE IS DONE!)
--------------------------------------------------------------------------------------------

  - dynamic updates of objects. TODO's:
    - Test and debug dynamic update (Sjoerd)
    - Map filter (Sjoerd)
  - following a person and fitting objects instead of GUI (?)
  - paramaterized models (Sjoerd)