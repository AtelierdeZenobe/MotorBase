# MotorBase

## Get the code

### Mbed Studio (Windows)
- Create a new mbed6 program (Shared instance: https://forums.mbed.com/t/how-to-create-an-instance-of-mbed-to-be-used-as-shared-instance-later/13261), give it a name of your choice (E.g. MotorBase)
- Open a terminal (Terminal -> new terminal)  
- In the terminal, init git and add the remote origin
```
cd MotorBase
git init
git remote add origin git@github.com:AtelierdeZenobe/MotorBase.git
```
- In the termininal, get the actual files from remote, and checkout the main branch
```
git pull origin main
```
If git cannot pull because of some local files, temporarily remove them, pull then bring them back.  

Now, all files must be present under MotorBase, in their respective folders.
- Remove the automatically created MotorBase/main.cpp. Mbed should now automatically track the correct files. 
- Get and init the necessary submodules
```
git submodule update --init
```
### Mbed CLI 2 (Linux)
```
mbed-tools new MotorBase
cd MotorBase
git init
git remote add origin git@github.com:AtelierdeZenobe/MotorBase.git
git fetch
git reset --hard origin/main
git submodule update --init
mbed-tools compile -m NUCLEO_F446RE -t GCC_ARM -f #Configure, compile, flash
```

Serial console:  
`sudo minicom -D /dev/ttyACM0`

## Build & run

`make`  
(compiles, flashes, opens minicom)  
see Makefile for specifc make options  

### On raspberry (ethercat master) side:
Rasp ip is 169.254.250.169  
`ethercat alias 69 -p 0 #If only one slave is connected, its ID should be 0`  
`./run.sh`

## Setup the hardware

TO UPDATE for MotorBase

![Schematic](./images/Servo42C_uart.png)

## Modify the code

- Create a new branch REMOTELY (easier than doing it locally and then pushing it)
https://github.com/AtelierdeZenobe/MotorBase/branches => New branch.
Give it a name, for example "newBranch". Use your currently checked out branch (main) as the source branch.

- In the terminal, pull the new branch and check it out
```
git pull origin newBranch
git checkout newBranch
```

- Modify or add files. When ready to add the files, add them to be commited
```
git add <filepath>
```

- Commit the changes (multiple files can be git add before commiting)
```
git commit -m "Description of the change"
``` 

- Push the changes to remote
```
git push
```

- Congratulation, you pushed your changes to the new branch remotely. Don't worry, you didn't change any thing in the main branch.

- Never use git merge to merge changes to the main. Instead, create a pull request from GitHub. And ask for help !

## Doc
[mbed events](https://os.mbed.com/blog/entry/Simplify-your-code-with-mbed-events/)

