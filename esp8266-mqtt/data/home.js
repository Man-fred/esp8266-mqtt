var indexData = null;
var reloadPeriod = 60000;
var running = false;

// reset known wlan from json object
function getSsid() {
	$.getJSON("ssid.json?"+ Date.now(),function(data){
		data.sort(function(a,b) {
			return b.RSSI - a.RSSI;
		});
		var radios = '';
		for (var i = 0; i < data.length; i++) {
			if (!radios.includes('"' + data[i].SSID + '"')) {
				radios += '<input  type="radio" name="ssid" id="ssid" value="' + data[i].SSID + '">'+data[i].SSID + ' (RSSI: ' + data[i].RSSI +') '+data[i].encryption+'<br />';
			}
		};
		if (i==0) {
			$("#AccessPoints").hide();
		} else {
			$("#noAccessPoints").hide();
		};
		$("div.listwlan").html(radios);
	});
	//$(".ssid").change()
}

function getConfig() {
	getSsid();

	$.getJSON("config.json?"+ Date.now(),function(data){
		$.each(data, function(name, val){
			var $el = $('[name="'+name+'"]'),
				type = $el.attr('type');

			switch(type){
				case 'checkbox':
					$el.attr('checked', 'checked');
					break;
				case 'radio':
					//$el.val(val);
					$el.filter('[value="'+val+'"]').attr('checked', 'checked');
					break;
				default:
					$el.val(val);
			};
		});
	});
}

function getIndex() {
	$.getJSON("index.json?"+ Date.now(), function(data) {
		$.each(data, function(id, val){
			$('[id="'+id+'"]').html(val);
			if (id == "myTitle") {
				document.title = val +"-Webserver";
			}
		});
		if(running) setTimeout(getIndex, reloadPeriod);
	});
}

function onBodyEdit(){
	var c={	};
	var d=window.location.href.replace(/[?&]+([^=&]+)=([^&]*)/gi,function(m,a,b){
		c[a]=b
	});
	var e=createEditor("editor",c.file,c.lang,c.theme);
	var f=createTree("tree",e);
	createFileUploader("uploader",f,e)
	// title and other fix parameters
	$.getJSON("index.json?"+ Date.now(),function(data){
		$.each(data, function(id, val){
			$('[id="'+id+'"]').html(val);
			if (id == "myTitle") {
				document.title = val +"-Webserver";
			}
		});
	});
};
function onBodyLoad() {
	var stopButton = document.getElementById("stop-button");
	stopButton.onclick = function(e){
		running = false;
	}
	var startButton = document.getElementById("start-button");
	startButton.onclick = function(e){
		run();
	}
	var refreshInput = document.getElementById("refresh-rate");
	refreshInput.value = reloadPeriod;
    refreshInput.onchange = function(e){
        var value = parseInt(e.target.value);
        reloadPeriod = (value > 0)?value:0;
        e.target.value = reloadPeriod;
    }
	getIndex();
	if ($("config").is(":visible") ) {
		getConfig();
	}
}
function set(page){
	function toggle(page, test){
		if (page == test) {
			$("#"+test).show(); 
			$("#m"+test).addClass("active");
		} else {
			$("#"+test).hide(); 
			$("#m"+test).removeClass("active");
		}
	}
	toggle(page, 'home');
	toggle(page, 'config');
	toggle(page, 'about');
	if ($("#config").is(":visible") ) {
		getConfig();
	}
}

    function run(){
      if(!running){
        running = true;
        getIndex();
      }
    }
