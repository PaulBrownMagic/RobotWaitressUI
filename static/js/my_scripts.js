var hiding;  // Used to log and cancel TimeOut functions

// Update values in menu
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

// Disable order button when all values are 0
function can_order() {
    var max = $(":input[type=number]").map(function(){ return this.value }).get().sort().reverse()[0]
    if (max == 0){  // max ordered is 0, therefore all values are 0
        $('#order').prop('disabled', true);
    }
    else {
        $('#order').prop('disabled', false);
    }
}
// Call can_order() on page load to disable button.
$(document).ready(function(){can_order();});


// Work the keypad for the login page
function pwd(num){
    var pswd = $("#login_pwd").val()
    $("#login_pwd").val(pswd + num)
}


// No Thanks button clicked
$("#decline").click(function(){
    // Time out Modal that pops up, it'll hide. Also return to hub called
    hiding = window.setTimeout(function () {
        $("#declineModal").modal("hide");
        $(location).attr('href',"/navigating/random");
    }, 5000);
});

// Modal from decline closed, cancel timeout
$('#dismiss').click(function(){
    clearTimeout(hiding);
    $(location).attr('href',"/navigating/random");
})

// User has accepted their order, POST url to set order status to complete
$(".complete").click(function(){
    var payload = {orderId: $("#orderId").html()}
    $.ajax({type: 'POST', url: "/order_complete", data: payload, success: function(result){
        console.log("Order status: Complete");
    }});
});

// Accepted order plus change to home page
$("#reorder").click(function(){
    var payload = {orderId: $("#orderId").html()}
    $.ajax({type: 'POST', url: "/order_complete", data: payload, success: function(result){
        console.log("Order status: Complete");
        $(location).attr('href',"/");
    }});
});
