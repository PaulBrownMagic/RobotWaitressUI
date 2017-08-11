$(document).ready(function() {
    // Open Websockets
    navsocket = io.connect(location.protocol + '//' + document.domain + ':' + location.port + '/nav');
    contentsocket = io.connect(location.protocol + '//' + document.domain + ':' + location.port + '/content');
    ordersocket = io.connect(location.protocol + '//' + document.domain + ':' + location.port + '/orders');

    contentsocket.on('new_content', function(content){
        $('#main_content').html(content)

        // When button id=decline is clicked:
        $('#decline').click(function(event){
            $("#declineModal").modal("show");
            // Time out Modal that pops up, it'll hide, robot is already moving away.
            window.setTimeout(function () {
                $("#declineModal").modal("hide");
                window.setTimeout(function(){
                    contentsocket.emit('home_nav', 'random');
                }, 500);
            }, 5000);
            return false;
        });

        $('#order').click(function(){
            ordersocket.emit('add', $("#order_form").serializeArray());
            contentsocket.emit('last_order');
        });
        $('#deliver').click(function(){
            $('#orderModal').modal('hide');
            window.setTimeout(function(){
                contentsocket.emit('deliver', $('#orderId').val());
            }, 500);
        });
        $('#cancel').click(function(){
            $('#orderModal').modal('hide');
            ordersocket.emit('cancel', $('#orderId').val());
            window.setTimeout(function(){
                contentsocket.emit('home');
            }, 500);
        });
        $('.complete').click(function(){
            ordersocket.emit('complete', $('#orderId').val());
        });
        $('.home').click(function(){
            contentsocket.emit('home');
        });
        $('#twitter').click(function(){
            contentsocket.emit('twitter');
        });
        $('#clear_goals').click(function(){
            navsocket.emit('clear_goals');
        });
    });
    $('#all_orders').click(function(){
        contentsocket.emit('all_orders');
    });
    $('#navigation').click(function(){
        contentsocket.emit('navigation');
    });
});


function nav_to(destination){
    contentsocket.emit('home_nav', destination);
}

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
        $('#order').attr('disabled', true);
    }
    else {
        $('#order').attr('disabled', false);
        console.log('false')
    }
}
