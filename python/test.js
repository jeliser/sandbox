async function userAction() {
    let response = await fetch('http://127.0.0.1:5000/execute?cmd=ls');
    let myJson = await response.json(); //extract JSON from the http response
    alert(myJson);
}
userAction();
