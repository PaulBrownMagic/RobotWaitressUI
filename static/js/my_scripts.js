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
    if (max == 0){
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

function reset_form(){
    // Reset form values to 0, used in menu
    $(":input[type=number]").val(0);
    $(".order_value").html("0");
    can_order();
}

// On click functions

// No Thanks button clicked
$("#decline").click(function(){
    // Reset form values to 0
    reset_form();
    // Time out Modal that pops up, it'll hide. Also return to hub called
    $.post({url: "return_to_hub", success: function(result){
        hiding = window.setTimeout(function () {
            $("#declineModal").modal("hide");
            $(location).attr('href',"/");
        }, 5000);
    }});
});

// Modal from decline closed, cancel timeout
$('#dismiss').click(function(){
    clearTimeout(hiding);
})

// User has accepted their order, POST url to set order status to complete
$(".complete").click(function(){
    var payload = {orderId: $("#orderId").html()}
    $.post({url: "/order_complete", data: payload, success: function(result){
        $(location).attr('href',"/");
    }});
});

// Call on page load, make sure menu items are 0
can_order();
