{/* <button onclick= "window.location.href = '/wifi_manager.html'"> Wifi Setup </button> */ }

let temporary_boolean = true;

async function on_Button_UP_pressed() {
  console.log(window.location.hostname)
  try {
    // const response = await fetch("http://nanostat.local/button1pressed", {
    const response = await fetch("/on_Button_UP_pressed", {
      method: "PUT",
      body: JSON.stringify({ on: temporary_boolean }),
      headers: {
        "Content-Type": "application/json",
      },
    });
  } catch (error) {
    alert("Request failed - check the console");
    console.error(error);
  }
}

async function on_Button_DOWN_pressed() {
  try {
    const response = await fetch("/on_Button_DOWN_pressed", {
      method: "PUT",
      body: JSON.stringify({ on: temporary_boolean }),
      headers: {
        "Content-Type": "application/json",
      },
    });
  } catch (error) {
    alert("Request failed - check the console");
    console.error(error);
  }
}

async function on_Button_STOP_pressed() {
  try {
    const response = await fetch("/on_Button_STOP_pressed", {
      method: "PUT",
      body: JSON.stringify({ on: temporary_boolean }),
      headers: {
        "Content-Type": "application/json",
      },
    });
  } catch (error) {
    alert("Request failed - check the console");
    console.error(error);
  }

}
async function on_Button_EMERGENCY_pressed() {
  try {
    const response = await fetch("/on_Button_EMERGENCY_pressed", {
      method: "PUT",
      body: JSON.stringify({ on: temporary_boolean }),
      headers: {
        "Content-Type": "application/json",
      },
    });
  } catch (error) {
    alert("Request failed - check the console");
    console.error(error);
  }
}


