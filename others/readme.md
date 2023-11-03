# LUA

### Coroutines
- Similar to threads in lua
- corioutine starts when explicity commanded to start and stops when explicity commanded to stop 
- coroutine share memory space while threads have their own memory space

### System calls in child script (not in main script)
- Only one main script exist 
- There can be many child scripts
1. sysCall_init
    - Called once when child script is called
2. sysCall_actuation
    - Called in each simulation pass
    - Handles all actuation functions
3. sysCall_sensing
    - Called in each simulation pass
    - Handles all sensing functions
4. sysCall_cleanup
    - Called once at end to restore all objects to its initial value

