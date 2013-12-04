function [phi,psi,inv_psi]=generateMatrixA(N,m,matrix_algorithm, basis_algorithm)
if nargin < 4
    basis_algorithm = 1;
end

switch basis_algorithm
    case 1
    %generate basis matrix
        disp('Using basis matrix: difference');
        inv_psi=zeros(N,N);
        for i=1:N
            t=zeros(1,N);
            if i~=N
                t(1,i)=-1;
                t(1,i+1)=1;
            else
                t(1,i)=-0.01;
            end
            inv_psi(i,:)=t;
        end
        psi=inv_psi^(-1);
    case 2
    %haar wavelet
        disp('Using basis matrix: Haar');
        % check input parameter and make sure it's the power of 2
        Level=log2(N);
        if 2^Level<N, error('please ensure the value of input parameter is the power of 2');end 

        %Initialization
        H=[1];
        NC=1/sqrt(2);%normalization constant
        %NC=1;
        LP=[1 1]; 
        HP=[1 -1];

        % iteration from H=[1] 
        for i=1:Level
            H=NC*[kron(H,LP);kron(eye(size(H)),HP)];
        end
        inv_psi=H;
        psi=inv_psi^(-1);
    case 3
    %generate basis matrix
        disp('Using basis matrix: DCT');
        inv_psi=dctmtx(N);
        psi=inv_psi^(-1);
    case 4
    %generate Daubechies 2 matrix
        disp('Using basis matrix: Daubechies-2');
        h0=(1+sqrt(3))/(4*sqrt(2));
        h1=(3+sqrt(3))/(4*sqrt(2));
        h2=(3-sqrt(3))/(4*sqrt(2));
        h3=(1-sqrt(3))/(4*sqrt(2));
        
        g0=h3;
        g1=-h2;
        g2=h1;
        g3=-h0;


        psi = zeros(N,N);
        for i=1:((N-4)/2+1)
            origin=(i-1)*2+1;
            psi(origin,origin+0) = h0;
            psi(origin+1,origin+0) = g0;
            psi(origin,origin+1) = h1;
            psi(origin+1,origin+1) = g1;
            
            if origin+3 <= N
                psi(origin,origin+2) = h2;
                psi(origin,origin+3) = h3;
                psi(origin+1,origin+2) = g2;
                psi(origin+1,origin+3) = g3;
            end
        end
        inv_psi=psi';
    case 5
    %generate Daubechies 8 matrix
        inv_psi = zeros(N,N);
        h= [-0.00011747678400228192 0.00067544940599855677 -0.00039174037299597711 -0.0048703529930106603 0.0087460940470156547 0.013981027917015516 -0.044088253931064719 -0.017369301002022108 0.12874742662018601 0.00047248457399797254 -0.28401554296242809 -0.015829105256023893 0.58535468365486909 0.67563073629801285 0.31287159091446592 0.054415842243081609];
        g= [ -0.054415842243081609 0.31287159091446592 -0.67563073629801285 0.58535468365486909 0.015829105256023893 -0.28401554296242809 -0.00047248457399797254 0.12874742662018601 0.017369301002022108 -0.044088253931064719 -0.013981027917015516 0.0087460940470156547 0.0048703529930106603 -0.00039174037299597711 -0.00067544940599855677 -0.00011747678400228192];            
        for i=1:((N-16)/2+1)
            origin=(i-1)*2+1;
            for j=1:8
                inv_psi(origin,origin+j-1) = h(1,j);
                inv_psi(origin+1,origin+j-1) = g(1,j);
            end
            
            if origin+15 <= N
                for j=9:16
                    inv_psi(origin,origin+j-1) = h(1,j);
                    inv_psi(origin+1,origin+j-1) = g(1,j);
                end
            end
        end
        psi=inv_psi';     
    case 6
        disp('Using basis matrix: identity');
        inv_psi=eye(N);
        psi=eye(N);
    case 7
        %generate basis matrix
        disp('Using basis matrix: DCT + hamming');
        h=hamming(N);
        filter_hamming = zeros(N,N);
        for i=1:N
            filter_hamming(i,i) = h(i);
        end
        inv_psi=filter_hamming*dctmtx(N);
        psi=inv_psi^(-1);
    case 8
        %generate average matrix
        disp('Using basis matrix: Averaging matrix');
        inv_psi=zeros(N,N);        
        k=4;
        pat=(1/k)*ones(1,2*k);
        pat(1,k+1:2*k) = pat(1,k+1:2*k)*(-1);
        %for i=1:N-2*k+1
        %    inv_psi(i,i:i+2*k-1) = pat;
        %end        
        %inv_psi(N-2*k+2:N,N-2*k+2:N) = eye(2*k-1);
        
        for i=1:2*k:N-2*k+1
            inv_psi(i,i:i+2*k-1) = pat;
            inv_psi(i+1:i+2*k-1,i+1:i+2*k-1) = eye( 2*k-1);
        end
        %disp(inv_psi);
        psi=inv_psi^(-1);
end

if nargin < 3
    matrix_algorithm = 1;
end

switch matrix_algorithm
    case 1
        %generate sampling matrix 'randomly'
        disp('Using measurement matrix: random-sparse');
        A=eye(m);
        B=randperm(N);
        B=logical(floor(B/(N-m+1)));
        phi=[];
        j=1;
        for i=1:N
            if B(1,i)==1
                phi=[phi,A(:,j)];
                j=j+1;
            else
                phi=[phi,zeros(m,1)];
            end
        end
    case 2        
        %generate sampling matrix 'uniformly'
        disp('Using measurement matrix: uniform-sparse');
        phi=zeros(m,N);
        acc=0;
        for i=1:m
            t=zeros(1,N);
            acc = acc + floor(N/m);
            t(1,acc) = 1;
            phi(i,:)=t;
        end
    case 3
        %random
        disp('Using measurement matrix: random_gaussian');
        %phi=rand(m,N);
        %for i=1:m
        %    phi(i,:)=randperm(N);
        %end
        phi = normrnd(0,sqrt(1/m),m,N);
    case 4
        disp('Using measurement matrix: random-binary');
        k = N*8/m;
        fprintf('Picking %f 1''s per row\n',k);
        for i=1:m
            phi(i,:) = 1*logical(floor(randperm(N)/(N-k)));
        end
    case 5
        disp('Using measurement matrix: random-binary-bern');
        phi = zeros(m,N);
        phi = binornd(1,0.5,m,N)*2 - 1;
        phi = phi * (1/sqrt(m));
    case 6
        disp('Using measurement matrix: sparse-8');
        matrix = gen_matrix_sparse(N,m,8,0);
        phi = matrix.A;        
    case 7
        disp('Using measurement matrix: sparse-16');
        matrix = gen_matrix_sparse(N,m,16,0);
        phi = matrix.A;        
    case 8
        disp('Using measurement matrix: sparse-32');
        matrix = gen_matrix_sparse(N,m,32,0);
        phi = matrix.A;
    case 9
        disp('Using measurement matrix: sparse-64');
        matrix = gen_matrix_sparse(N,m,64,0);
        phi = matrix.A;        
end
%A=phi*psi;


end
