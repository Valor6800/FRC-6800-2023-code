## Making a new auto command
### 1. Make sure you actually need it
Asks yourself this: can you do this thing in teleop? If the answer is yes, then unless it requires human observation (like balancing) or it takes a variable amount of time to complete (like with elevarm), you do not need a new command. No, you don't need a new command. I can promise you don't need a command. I'm telling you, you don't need a new command. Again, you don't need a new command. Did I get that across?

If you don't need a new command, go [here](#if-you-dont-need-a-new-command).
If you do, keep reading.

### 2. Adding a ValorAutoAction
First, go to [ValorAutoAction.h](../../include/auto/ValorAutoAction.h) and a new type
```cpp
enum Type {
    ...
    ELEVARM,
    ACCELERATION,
    AWESOME_COMMAND
}
```
then go to [ValorAutoAction.cpp](ValorAutoAction.cpp). Add a string-to-command type conversion
```cpp
...
} else if (items[0] == "acceleration" || items[0] == "accel") {
    type = ValorAutoAction::Type::ACCELERATION;
} else if (items[0] == "awesome"){
    type = ValorAutoAction::Type::AWESOME_COMMAND;
}
```
and go to the end to add some logic. This is where you'll take the list of strings (the comma separated values) and parse them into usable numbers, booleans, etc. Store these in variables already present and appropriately named in ValorAutoAction.h. **Look through *all* the variables before adding a new one.** 
```cpp
...
else if (type == ValorAutoAction::Type::AWESOME_COMMAND)
{
    // items[0] is the name of the command that was entered
    awesomeness = stod(items[1]) + stod(items[2]);
}
```
You'll also want to add some logic for processing errors in user input. Most commonly, that'll just be `SIZE_MISMATCH` but feel free to add more elaborate error types. If you want to do so, add to the `enum Error`, then go to [ValorAuto.h](../../include/auto/ValorAuto.h) and add to the `errorToStringMap`.

```cpp
...
if (items.size() < 3){
    error = ValorAutoAction::Error::SIZE_MISMATCH;
    return;
}

awesomeness = stod(items[1]) + stod(items[2]);
if (awesomeness < 999){
    error = ValorAutoAction::Error:NOT_AWESOME_ENOUGH;
}
```
That's all you need to do here. The purpose of `ValorAutoAction` isn't to do any logic - that's all done in `ValorAuto` - it just stores some numbers. 

### 3. Adding your action to ValorAuto
Alright, now that you've got some data in an object, lets use it. 

Go to [ValorAuto.cpp](ValorAuto.cpp) and add a check to the end of `makeAuto`, this is where you'll add your logic. 
```cpp
... 
else if (action.type == ValorAutoAction::ELEVARM){ // you don't have to put the enum type when specifying the enum as you might've noticed. I still do it for the sake of clarity
    ...
}
else if (action.type == ValorAutoAction::Type::AWESOME_COMMAND){
    // [logic]
}
```
Here, you'll add your command to the sequence 

```cpp
// [logic]:
currentGroup->AddCommands(
    frc2::InstantCommand(
        [&, action] {
            if (action.awesomeness > 10000){
                table->PutString("awesomeness", "really awesome");
                lights->FlashEmGreen();
            }
            else {
                lights->FlashEmRed();
            }
        }
    )
);
```
What you put into the sequence is 100% up to you. As long as the command finishes, you're welcome to add it to the sequence.

#### **Adding a command for a new subsystem**
Most commonly, when adding auto functionality for a subsystem, you'll just get the auto command you made over there (as a pointer) and add it to the group. Here's how you do that:
```cpp
currentGroup->AddCommands(
    std::move(*awesominator->getAutoCommand(
        action.awesomeness
    ))
);
```
If you're doing so, make sure you're adding the subsystem to the auto in [Robot.cpp](../Robot.cpp) at the top
```cpp
autonomous(&drivetrain, &intake, &elevarm, &awesominator)
```
add it to the auto class at [ValorAuto.h](../../include/auto/ValorAuto.h)
```cpp
...
Intake *intake;
Elevarm *elevarm;
Awesominator *awesominator;
```
and initialize it in [ValorAuto.cpp](ValorAuto.cpp) at the top
```cpp
... intake(_intake), elevarm(_elevarm), awesominator(_awesominator)
```

**An important note:** Your logic inside `makeAuto` **runs during compile time** (i.e. when you hit enable), **not during run time**. All you're meant to do here is add a command to the sequence with the stuff you want to do, not the actual stuff. You're welcome to put in debug messages, but if you were to, for example, set the elevarm state outside of a command, it would do all of those right when you hit enable, so it would just go to the last state you told it to go immediately. 

### If you don't need a new command
The most common alternative to creating an entire command and cluttering up my precious beautiful baby boy is to use `state`. 
To do this, first navigate to the `state` logic in [ValorAuto.cpp](ValorAuto.cpp) 
```cpp
if (action.type == ValorAutoAction::Type::STATE){
    std::function<void(void)> func;
    ...
    currentGroup->AddCommands(frc2::InstantCommand(func));
}
```
and add in your state setting logic 

```cpp
if (action.type == ValorAutoAction::Type::STATE){
    std::function<void(void)> func;
    ...
    if (action.state == "bean"){
        func = [&, action] {
            bean->state.beanState = bean->stringToBeanState(action.value);
        };
    }
    ...
    
    currentGroup->AddCommands(frc2::InstantCommand(func));
}
```
You'll notice that we use a `stringToState` function that probably doesn't exist yet. Go to your subsystem header file and add it

```cpp
// bean.h

public:
...
BeanState stringToBeanState(std::string name){
    return stringToBeanStateMap[name];
}
```
Now we're using a map that doesn't exist. Add it to the header file. Keep in mind that the name you give to the enum in string form doesn't have to exactly match, it's just helpful if you do. 
```cpp
std::unordered_map<std::string, BeanState> beanStateMap = {
    "beaning": BeanState::BEANING,
    "otherthing": BeanState::NOT_BEANING
};
BeanState stringToBeanState(std::string name){
    return stringToBeanStateMap[name];
}
```
And that's it.

Now, that may not be enough. If you're doing several commands in a row, you can turn that into an action. Read the [auto command docs](../../../../scripts/auto/readme.md#action) to learn more about them.