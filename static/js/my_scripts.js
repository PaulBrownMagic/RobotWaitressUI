var hiding;

// The function that actually updates the values
function change(diff, id){
    var num = parseInt(document.getElementById("f "+id).value);
    if (num + diff <= 5 &&
        num + diff >= 0 ){
        num = num + diff;
        document.getElementById("f "+id).value = num;
        document.getElementById("s "+id).innerHTML = num;
    }
    can_order();
}

function pwd(num){
    var pswd = $("#login_pwd").val()
    $("#login_pwd").val(pswd + num)
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

function reset_form(){
    // Reset form values to 0
    $(":input[type=number]").val(0);
    $(".order_value").html("0");
    can_order();
}

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

$('#dismiss').click(function(){
    clearTimeout(hiding);
})

$(".complete").click(function(){
    var payload = {orderId: $("#orderId").html()}
    $.post({url: "/order_complete", data: payload, success: function(result){
        $(location).attr('href',"/");
    }});
});

// Call on page load
can_order();
