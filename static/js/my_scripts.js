var navsocket;
var contentsocket;
var ordersocket;

$(document).ready(function() {
    // Open Websockets
    navsocket = io.connect(location.protocol + '//' + document.domain + ':' + location.port + '/nav');
    contentsocket = io.connect(location.protocol + '//' + document.domain + ':' + location.port + '/content');
    ordersocket = io.connect(location.protocol + '//' + document.domain + ':' + location.port + '/orders');
    // Extend timeouts
    navsocket.heartbeatTimeout = 60000 * 15;
    contentsocket.heartbeatTimeout = 60000 * 15;
    ordersocket.heartbeatTimeout = 60000 * 15;

    // Load new HTML into the DOM, reload these functions to work with new DOM elements.
    contentsocket.on('new_content', function(content){
        $('#main_content').html(content);

        // When decline button is clicked:
        $('#decline').click(function(){
            $("#declineModal").modal("show");
            // Time out Modal that pops up, it'll hide, robot is already moving away.
            window.setTimeout(function () {
                $("#declineModal").modal("hide");
                window.setTimeout(function(){
                    contentsocket.emit('home_nav', 'choose');
                }, 500);
            }, 5000);
            return false;
        });

        // Submit order form.
        $('#order').click(function(){
            ordersocket.emit('add', $("#order_form").serializeArray());
            contentsocket.emit('last_order');
        });

        // Request order delivery.
        $('#deliver').click(function(){
            $('#orderModal').modal('hide');
            window.setTimeout(function(){
                contentsocket.emit('deliver', $('#orderId').val());
            }, 500);
        });

        // Request order status is updated to 'Cancelled'.
        $('#cancel').click(function(){
            $('#orderModal').modal('hide');
            ordersocket.emit('cancel', $('#orderId').val());
            window.setTimeout(function(){
                contentsocket.emit('home');
            }, 500);
        });

        // Request order status is updated to 'Complete'.
        $('.complete').click(function(){
            ordersocket.emit('complete', $('#orderId').val());
        });

        // HTML content navigation: Home page "/"
        $('.home').click(function(){
            contentsocket.emit('home');
        });

        // HTML content navigation: Twitter page "/twitter"
        $('#twitter').click(function(){
            contentsocket.emit('twitter');
        });

        // Request all current navigation goals are cleared.
        $('#clear_goals').click(function(){
            navsocket.emit('clear_goals');
        });
    });

    // Functions that are not reloaded with new content:

    // HTML content navigation: All Orders page "/all_orders"
    $('#all_orders').click(function(){
        contentsocket.emit('all_orders');
    });

    // HTML content navigation: Navigation page "/navigation"
    $('#navigation').click(function(){
        contentsocket.emit('navigation');
    });

    // Display information provided within a pop-up modal.
    contentsocket.on('info', function(info){
      $('#info_title').html(info['title']);
      $('#info_text').html(info['text']);
      $('#infoModal').modal().show()
    });
});

// Request navigation to a location.
function nav_to(destination){
    contentsocket.emit('home_nav', destination);
}

// Update values in menu, used in Home page.
function change(diff, id){
    var formid = "[id='f "+ id + "']";
    var showid = "[id='s "+ id + "']";
    var num = parseInt($(formid).val(), 10);
    if (num + diff <= 5 && num + diff >= 0 ){
        num = num + diff;
        $(formid).val(num);
        $(showid).html(num);
    }
    can_order();
}

// Disable order button when all values are 0, used in Home page.
function can_order() {
    var max = $(":input[type=number]").map(function(){ return this.value }).get().sort().reverse()[0];
    if (max == 0){  // max ordered is 0, therefore all values are 0
        $('#order').attr('disabled', true);
    }
    else {
        $('#order').attr('disabled', false);
    }
}
