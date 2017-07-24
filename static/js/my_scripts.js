var hiding;  // Used to log and cancel TimeOut functions

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

function pwd(num){
    // Work the keypad for the login page
    var pswd = $("#login_pwd").val()
    $("#login_pwd").val(pswd + num)
}


// On click functions
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

$("#reorder").click(function(){
    var payload = {orderId: $("#orderId").html()}
    $.ajax({type: 'POST', url: "/order_complete", data: payload, success: function(result){
        console.log("Order status: Complete");
        $(location).attr('href',"/");
    }});
});

// Call on page load, make sure menu items are 0
can_order();
