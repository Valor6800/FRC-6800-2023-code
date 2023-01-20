# How to Add Dashboard Values to a Subsytem

## Step 1: In the header, extend Sendable and SendableHelper:

```
    class NameOfSubsystem : public ValorSubsystem, public wpi::Sendable, public wpi::SendableHelper<NameOfSubsystem>
```

**Don't forget to change NameOfSubsystem to the name of YOUR subsystem**


## Step 2: In the header, in public, create the decleration for the InitSendable function:

```
    void InitSendable(wpi::SendableBuilder& builder) override;
```


## Step 3: In the cpp, create the definition for the InitSendable function:

```
    void NameOfSubsystem::InitSendable(wpi::SendableBuilder& builder)
    {
        builder.SetSmartDashboardType("Subsystem");
    }
```


## Step 4: Add values through addProperty in InitSendable. 
**Replace NameOfValue with a string and value with your chosen property type.**

<table>
<tr>
<td> Type </td> <td> Implementation </td>
</tr>
<tr>
<td> Double </td>
<td>

```
    builder.AddDoubleProperty(
        "NameOfValue",
        [this] { return value; },
        nullptr
    );
```

</td>
</tr>
<tr>
<td> Boolean </td>
<td>

```
    builder.AddBooleanProperty(
        "NameOfValue",
        [this] { return value; },
        nullptr
    );
```

</td>
</tr>
<tr>
<td> String </td>
<td>

```
    builder.AddStringProperty(
        "NameOfValue",
        [this] { return value; },
        nullptr
    );
```

</td>
</tr>
<tr>
<td> Double Array </td>
<td>

```
    builder.AddDoubleArrayProperty(
        "NameOfValue",
        [this] { return value; },
        nullptr
    );
```

</td>
</tr>
<tr>
<td> Boolean Array </td>
<td>

```
    builder.AddBooleanArrayProperty(
        "NameOfValue",
        [this] { return value; },
        nullptr
    );
```

</td>
</tr>
<tr>
<td> String Array </td>
<td>

```
    builder.AddStringArrayProperty(
        "NameOfValue",
        [this] { return value; },
        nullptr
    );
```

</td>
</tr>
</table>
