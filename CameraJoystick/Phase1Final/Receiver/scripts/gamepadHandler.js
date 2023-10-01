import { switchView, toggleRotation, modeSet, moveDot, sendDotCoordinates } from './UI.js';

const gamepads = {};
const gamepads2 = {};
let updateLoopInterval;
let autoModeLoopInterval;
let lastButtonPressTime = 0;

function handleGamepadConnected(event) {
    const gamepad = event.gamepad;
    console.log(`Gamepad connected: ${gamepad}`);
    gamepads[gamepad.index] = gamepad;
}

function handleGamepadDisconnected(event) {
    const gamepad = event.gamepad;
    console.log(`Gamepad disconnected: ${gamepad}`);
    delete gamepads[gamepad.index];
}

function isButtonDebounced() {
    const currentTime = Date.now();
    const timeDifference = currentTime - lastButtonPressTime;
  
    if (timeDifference < 300) { // assuming 300ms as the threshold
      return false;
    }
  
    lastButtonPressTime = currentTime;
    return true;
  }
  
  
function toggleAutoMode(dataChannel) {        
    console.log('AutoMode Enabled');
    videoContainer.classList.remove('view1', 'view2', 'view3');
    videoContainer.classList.add('view3');
    dot.classList.remove('dotHidden');
    dot.classList.add('dotNotHidden');
    modeSet("AutoMode");
        clearInterval(updateLoopInterval); // Clear the interval to stop the loop
        autoModeLoopInterval = setInterval(() => {
            const gamepadList2 = navigator.getGamepads();
            for (let i = 0; i < gamepadList2.length; i++) {
                const gamepad2 = gamepadList2[i];
                if (gamepad2) {
                    if (!gamepads2[gamepad2.index] || gamepad2.timestamp !== gamepads2[gamepad2.index].timestamp) {
                        console.log(`Automode Pointer Moving`, gamepad2);
                        gamepads2[gamepad2.index] = gamepad2;
                        
                        const sensitivity = 30; // Adjust as needed
                        let x = (gamepad2.axes[0] * sensitivity);
                        let y = (gamepad2.axes[1] * sensitivity);
                        moveDot (x,y);
                        
                        if (gamepad2.buttons[7] && gamepad2.buttons[7].pressed && isButtonDebounced()) {
                            console.log('AutoMode Disabled');
                            dot.classList.remove('dotNotHidden');
                            dot.classList.add('dotHidden');                        
                            clearInterval(autoModeLoopInterval); // Clear the interval to stop the loop
                            startGamepadUpdates(dataChannel);
                        } else if (gamepad2.buttons[0] && gamepad2.buttons[0].pressed && isButtonDebounced()) {
                            sendDotCoordinates (dataChannel);
                        }
                    }
                } else if (gamepads2[i]) {
                    console.log(`Gamepad ${i} disconnected`);
                    delete gamepads2[i];
                }
            }
        }, 100); // Every 100 milliseconds
    }


export function startGamepadUpdates(dataChannel) {
    modeSet("ManualMode");
    updateLoopInterval = setInterval(() => {
        const gamepadList = navigator.getGamepads();
        for (let i = 0; i < gamepadList.length; i++) {
            const gamepad = gamepadList[i];
            if (gamepad) {
                if (!gamepads[gamepad.index] || gamepad.timestamp !== gamepads[gamepad.index].timestamp) {
                    console.log(`Gamepad ${i} status changed:`);
                    gamepads[gamepad.index] = gamepad;
                    const gamepadData = {
                        timestamp: gamepad.timestamp,
                        axes: gamepad.axes,
                        buttons: gamepad.buttons.map(button => ({ pressed: button.pressed, value: button.value }))
                    };
                    if (gamepad.buttons[2] && gamepad.buttons[2].pressed && isButtonDebounced()) {
                        switchView();
                    } else if (gamepad.buttons[3] && gamepad.buttons[3].pressed && isButtonDebounced()) {
                        toggleRotation();
                    } else if (gamepad.buttons[7] && gamepad.buttons[7].pressed && isButtonDebounced()) {
                        toggleAutoMode (dataChannel);
                    } else {
                    const json = JSON.stringify(gamepadData);
                    dataChannel.send(json); // Send gamepad data to the other peer
                }
                }
            } else if (gamepads[i]) {
                console.log(`Gamepad ${i} disconnected`);
                delete gamepads[i];
            }
        }
    }, 100); // Every 100 milliseconds
}


window.addEventListener('gamepadconnected', handleGamepadConnected);
window.addEventListener('gamepaddisconnected', handleGamepadDisconnected);