# Rocket Model Example
An example rocket consists of the following models.
* Airframes (multiple per stage)
* Flight Computer
* Flight Software
* Tanks
  * Liquid Propellant Tanks
  * Solid Propellant "Tanks"
  * Gaseous Tanks (i.e. Nitrogen/Helium)
* Sensors
  * Pressure Sensors (i.e. Ullage Pressure)
  * Delta-Pressure Sensors (i.e. Differential pressure between a liquid propellant tank ullage and liquid head)
  * Accelerometers
  * Gyros
  * etc.
* Engines
  * Liquid Engines (draws from one or more tanks)
    * Stage engines (e.g. uses a fuel and oxidizer from respective tanks)
    * RCS engines (e.g. uses hydrazine from a hydrazine tank)
  * Solid Rocket Engines (e.g. uses solid rocket fuel)
  
Rockets usually have a `Vehicle` composed of one or more stages
Stages are jettisoned at specific events.
Engines are jettisoned at specified events (i.e. solid rocket boosters)
`Stages`'s are usually composed of one or more `Engine`s, `Tank`s, and `Airframe`, a `Computer`, `Sensor`s, etc.
Models need to "talk" to other models. This is the tricky part.
  * E.g. engines burn propellant which drains tanks (Tank <-> Engine, typically a many engines to many tank relationship)
  * Sensors sense state of any model
  * Some ownership is shakey (i.e. total vehicle acceleration is computed from total engine thrust and total vehicle mass)
  * `Computer` model will also need to read `Sensor` states so it can provide new commands to any vehicle model.
  * Flight Computer can run many different flight software models. I'd assume a abstract flight sofware interface to keep things manageable.
This is an engineering time-domain simulation using boost::numeric::odeint.
Since models talk with one another we need a good flow here so models do not get into a circular dependency loop trying to update themselves
requiring information from other models.
We need a method to keep us from corrupting ODE steppers derivative states during integration steps via model state updates.
Flight Computers and the contained Flight Software must run at discrete fixed rates (i.e. 100 Hz)
We need a method of making it easy to telemeter information from all the models to an output file.
The user supplies the vehicle configuration, simulation parameters, and model connections via input file.
This means the simulation does not know the vehicle build-up until runtime.
