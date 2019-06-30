# Responsible: Arpit

## CHALLENGE SERVING DRINKS

## Scenario (RWC2019)

I Start:

    The robot starts at a designated starting position <serving_drinks_initial> (in the hallway) and drives to the living room.
    To qualify for the bonus reward, first go to inspect the bar and then drive to the living room.

II Detection / Get order:

    Setup:
        There are at least 5 guests in the living room, two standing and three sitting.
        Each guest has assigned a predefined name and has either a drink or a drink request (choice and alternative).
    Procedure:
        The robot detects which are the people without a drink and then gets order.
        The get order procedure includes:
            - drive to person
            - asks for his/her name
            - learn operator (to recognize him later)
            - get order for a drink
                *In case the ordered drink is unavailable, the robot announces that and asks for alternative choice.

III Waiter duty:

    Setup:
        The bar can be any flat surface where objects can be placed, in any other room but the living room.
            *2h before test the bar location is specified and announced
        The Bartender may be standing either behind the bar or next to it, depending on the arena setup.
        All available beverages are on top of the bar.
        One of the drink requests is not available.
    Procedure:
        The robot drives to the bar, grabs the correct drink and returns to the living room.
        The robot finds the person, who requested the drink and hand it over.

        *The robot may either hand-over drinks orhave attached a tray.
        *When a tray is used, the robot must be sure that the guest is taking the correct drink (guests may try to take the wrong one).
        *After giving the order (when the robot is not in the living room), the referees may re-arrange the people.


## Scoring sheet of RGO 2019:

    *Partial scoring applies - we get points per served drink
    *We get bonus rewards only if (at least) one main goal is successfuly executed

    OC instructions:
        2h before test specify and announce the bar location!

    The maximum time for this test is 5 minutes!

    Main Goal:
        Deliver a drink to a guest:                                 3 x 250

        Penalties:
            Each drink handed-over to the robot (bypass picking)    3 × –75
            Each drink taken by a guest (bypass drink handover)     3 × –75
            Each guest approaching to the robot to place order      3 × –100
            Each guest waving or calling the robot to place order   3 × –50
            Telling the robot which drink is unavailable            2 × –100

        Bonus rewards:
            Inform a guest of drink unavailability upon request     250

    Total score (excluding penalties and standard bonuses):         1000

    Outstanding performance:                                        100


## Testing / Running

Launch procedure:
    ```
    robot-start
    ```
    ```
    robot-challenge-serving-drinks
    ```

## ToDo:

    Status after RGO2019:

        - After the initial inspection we ask for the unavailable drink.
        - Should be checked if the passing of the inspected objects is correct and it works.
        - Waving detection implemented, should be tested.
        - Detection of people, holding drinks not yet implemented. Is it necessary?

    Enchancements:

        - Storing info for the available drinks (implemented, not tested)
        - Detect people holding drinks

    Tested successfully:

        - Learn people and their names
        - Inspecting bar
        - Grasping a drink (handover from human)
        - Find specific person in the room stating his/her name and requested drink
        - Hand over drink
        - Run full challenge :)
