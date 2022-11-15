# How to Add Dashboard Values to a Susbsytem

##Step 1: in the header, extend Sendable and SendableHelper:

    'class NameOfSubsystem : public ValorSusbsystem, public wpi::Sendable, public wpi::SendableHelper<NameOfSubsystem>'

==Don't forget to change NameOfSubsystem to the name of YOUR subsystem== 


##Step 2: in the header, in public, create the decleration for the InitSendable function:

    'void InitSendable(wpi::SendableBuilder& builder) override;'


##Step 3: in the cpp, create the definition for the InitSendable function:

    ```
    void NameOfSubsystem::InitSendable(wpi::SendableBuilder& builder)
    {
        builder.SetSmartDashboardType("Susbsystem");
    }
    ```


##Step 4: add values through addProperty in InitSendable. ==Replace NameOfValue with a string and value with your chosen property type.==

###For a Double:

    ```
    builder.AddDoubleProperty(
        "NameOfValue",
        [this] { return value; },
        nullptr
    );
    ```

###For a Boolean:

    ```
    builder.AddBooleanProperty(
        "NameOfValue",
        [this] { return value; },
        nullptr
    );
    ```

###For a String:

    ```
	builder.AddStringProperty(
        "NameOfValue",
        [this] { return value; },
        nullptr
    );
    ```

###For a Double Array:

    ```
    builder.AddDoubleArrayProperty(
        "NameOfValue",
        [this] { return value; },
        nullptr
    );
    ```

###For a Boolean Array:

    ```
    builder.AddBooleanArrayProperty(
        "NameOfValue",
        [this] { return value; },
        nullptr
    );
    ```

###For a StringArray:

    ```
	builder.AddStringArrayProperty(
        "NameOfValue",
        [this] { return value; },
        nullptr
    );
    ```