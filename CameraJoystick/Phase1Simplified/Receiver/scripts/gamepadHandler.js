import { switchView, toggleRotation } from './UI.js';

const gamepads = {};

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



export function startGamepadUpdates(dataChannel) {
    setInterval(() => {
        const gamepadList = navigator.getGamepads();
        for (let i = 0; i < gamepadList.length; i++) {
            const gamepad = gamepadList[i];
            if (gamepad) {
                if (!gamepads[gamepad.index] || gamepad.timestamp !== gamepads[gamepad.index].timestamp) {
                    console.log(`Gamepad ${i} status changed:`);
                    gamepads[gamepad.index] = gamepad;
                    const gamepadData = {
                        id: gamepad.id,
                        timestamp: gamepad.timestamp,
                        axes: gamepad.axes,
                        buttons: gamepad.buttons.map(button => ({ pressed: button.pressed, value: button.value })),
                        connected: gamepad.connected
                    };
                    if (gamepad.buttons[2] && gamepad.buttons[2].pressed) {
                        switchView();
                    }            
                    if (gamepad.buttons[3] && gamepad.buttons[3].pressed) {
                        toggleRotation();
                    }
                    const json = JSON.stringify(gamepadData);
                    dataChannel.send(json); // Send gamepad data to the other peer
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