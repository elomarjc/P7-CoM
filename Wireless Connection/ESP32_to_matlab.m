clear all
close
x = serialport('COM5',115200,"StopBits",1,"Parity","none","DataBits",8,"FlowControl","none")

start = tic;
n_comms = 0;
test_time = 1;
while toc < test_time
    %a(n_comms+1) = typecast(uint8(fread(x,4)),'single');
    if(x.NumBytesAvailable == 0)
        continue
    end
    read = fgets(x)
    json_struct = jsondecode(read)
    n_comms= n_comms+1;
    %message = convertCharsToStrings(native2unicode(a))
    %typecast(a,'single')
    
    ellapsed = toc
end
disp(sprintf("The frequency is %f",n_comms/test_time))

