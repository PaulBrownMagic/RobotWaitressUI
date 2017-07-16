var socket; // The websocket, made global.
var hiding;  // Used to log and cancel TimeOut functions

// Modal from decline closed, cancel timeout
$('#dismiss').click(function(){
    clearTimeout(hiding);
})

// Display Modal to ask for Help with navigation. Template in "base.html"
function display_helper(msg) {
    $('#help_title').html(msg.title);
    $('#help_text').html(msg.text);
    if(typeof msg.button !== 'undefined'){
    $('#help_button').html(msg.button);
    }
    else {
        $('#help_container').html("")
    }
    $('#helpModal').modal("show");
}

// Request navigation to WayPoint, change back to homescreen as robot is moving
function nav_to(waypoint){
    socket.emit('go_to', {destination: waypoint});
    $(location).attr('href',"/");
};

// Update values in menu, used in "home.html"
function change(diff, id){
    var formid = "[id='f "+ id + "']"
    var showid = "[id='s "+ id + "']"
    var num = parseInt($(formid).val(), 10);
    if (num + diff <= 5 && num + diff >= 0 ){
        num = num + diff;
        $(formid).val(num);
        $(showid).html(num);
    }
    can_order();
}

// Disable order button when all values are 0, used in "home.html"
function can_order() {
    var max = $(":input[type=number]").map(function(){ return this.value }).get().sort().reverse()[0]
    if (max == 0){  // max ordered is 0, therefore all values are 0
        $('#order').prop('disabled', true);
    }
    else {
        $('#order').prop('disabled', false);
    }
}
// Call on page load, make sure menu items are 0
can_order();

// Work the keypad for the login page, "login.html"
function pwd(num){
    var pswd = $("#login_pwd").val()
    $("#login_pwd").val(pswd + num)
}
