var socket; // The websocket, made global.
var hiding;  // Used to log and cancel TimeOut functions
// Modal from decline closed, cancel timeout
$('#dismiss').click(function(){
    clearTimeout(hiding);
})

function display_helper(msg) {
    $('#help_title').html(msg.title);
    $('#help_text').html(msg.text);
    if(typeof msg.button !== 'undefined'){
    $('#help_button').html('<button type="button" class="btn btn-default" data-dismiss="modal">'+msg.button+'</button>');
    }
    $('#helpModal').modal("show");
}

function change(diff, id){
    // Update values in menu
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

function can_order() {
    // Disable order button when all values are 0
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

function pwd(num){
    // Work the keypad for the login page
    var pswd = $("#login_pwd").val()
    $("#login_pwd").val(pswd + num)
}
