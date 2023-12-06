clear all
clc
x = serialport('COM4',115200,"StopBits",1,"Parity","none","DataBits",8,"FlowControl","none")

%%
msg = zeros(1,4);

tic
while true
    %a(n_comms+1) = typecast(uint8(fread(x,4)),'single');
    if(x.NumBytesAvailable >=3 )
        x.NumBytesAvailable;
        read = readline(x);
        i = 1+i;
        read = read(i:end);
        try
        json_struct = jsondecode(read);
        catch 
        json_struct.msg = msg(:,end);
      
        end
      msg = json_struct;
    end

    if toc > 1
        disp(msg);
        tic
    end
    %message = convertCharsToStrings(native2unicode(a))
    %typecast(a,'single')
end
