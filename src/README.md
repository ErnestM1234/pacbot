#Quick rundown

Basically there are three entities that are going to be communicating:
(1) Server
(2) Pacbot
(3) Game Engine
I apologize in advanced for the naming conventions because it may (will) get a little confusing.

Comms Diagram:
(Server) ---> (Pacbot) <---> (Game Engine)

(1) Server:
The Server represents the entity that will feed the pacbot information about the state of the game,
including the pacbot's position in the grid, where the ghosts are, etc. The server is hosted by
Harvard at the competition. This is (confusingly) stored in the folder titled gameEngine.

(2) Pacbot:
The Pacbot is an entity that will be run on our Pacbot. It receives information from Server and
transmits the info to Game Engine.

(3) Game Engine:
Game Engine is the entity that will be run on a PC and communicates with Pacbot. It receives information
about the Pacbot and will return the information needed for the Pacbot to make the next action.