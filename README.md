# Notes2Robotics

```puml
@startuml
!pragma useVerticalIf on
start
if (条件 A) then (yes)
  :文本 1;
elseif (条件 B) then (yes)
  :文本 2;
  stop
elseif (条件 C) then (yes)
  :文本 3;
elseif (条件 D) then (yes)
  :文本 4;
else (nothing)
  :文本 else;
endif
stop
@enduml
```

