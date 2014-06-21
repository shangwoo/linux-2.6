// It is NOT necessary to modify this module
// Keypad can be configured in menuconfig and core.c
// zhaoy@alpsacle.com

#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/types.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/errno.h>

#include <asm/irq.h>
#include <asm/io.h>
#include <asm/mach-types.h>

#include <mach/keypad.h>
#include <mach/hardware.h>
#include <mach/pincontrol.h>
#include <mach/as3310x_monitor.h>
#include <mach/pwm.h>




#ifdef CONFIG_ASAP18XX_KEYPAD_DBG
    #define keypaddubug 1
#else// CONFIG_ASAP18XX_KEYPAD_DBG
    #define keypaddubug 0
#endif// CONFIG_ASAP18XX_KEYPAD_DBG

static void as3310_kp_timer_tingle(unsigned long);

static struct as3310_kp{
    struct input_dev *input;
    struct timer_list timer_tingle;
    int irq;
    int irq_flag;
    int irq_flag_new;
    int irq_flag_old;
    int last_3times;
    int ready2finish;
    int functionkey_flag_start;
    int functionkey_flag_F1;
    int functionkey_flag_F2;
    struct as3310_kp_special_key special_key;
    struct as3310_kp_gpios_row gpio_rows[ROW];
    struct as3310_kp_gpios_col gpio_cols[COL];
#if keypaddubug
    int key_map[ROW][COL];
#endif
    int delay;
    char keypad_state[ROW][COL];//0=up;1=down
    char keypad_state_new[ROW][COL];
    char keypad_state_count[ROW][COL];//the value of state should be scaned twice to avoid the shake.
    int *keymap;
};
struct as3310_kp *as3310_kp;

struct as3310_kp_platform_data *pdata;

#if keypaddubug
static void printstate(void){
    int i,j;
    printk("====state=======\n");
    for(i=0;i<ROW;i++){
        for(j=0;j<COL;j++)
            printk(" %d ",as3310_kp->keypad_state[i][j]);
        printk("\n");
    }
    printk("====state_new=======\n");
    for(i=0;i<ROW;i++){
        for(j=0;j<COL;j++)
            printk(" %d ",as3310_kp->keypad_state_new[i][j]);
        printk("\n");
    }
}
#endif

#if 0//CONFIG_AS3310x_MONITOR
static void combinekey(){
    if(as3310_kp->keypad_state[0][1]==1){//start +
        if(as3310_kp->keypad_state[0][0]==1){
            //printk("~~~");
          //  vol_change(VOLUME_UP);
        }
    }
    if(as3310_kp->keypad_state[1][2]==1){//start -
        if(as3310_kp->keypad_state[0][0]==1){
            //printk("~~~");
        //    vol_change(VOLUME_DOWN);
        }
    }
    if(as3310_kp->keypad_state[0][2]==1){//start <
        if(as3310_kp->keypad_state[0][0]==1){
            //printk("~~~");
            change_one_step(PWM_BACKLIGHT,BL_UP);
        }
    }
    if(as3310_kp->keypad_state[1][1]==1){//start >
        if(as3310_kp->keypad_state[0][0]==1){
            //printk("~~~");
            change_one_step(PWM_BACKLIGHT,BL_DOWN);
        }
    }
}
#endif


static void kp_check_finish(void){// return 1 if all empty
int j,voltage;

    as3310_kp->ready2finish = 1;

    voltage = read_GPIO(as3310_kp->special_key.port,
                    as3310_kp->special_key.pin);

    if (voltage == 1) as3310_kp->ready2finish = 0;//its being pressed


    for(j=0;j<COL;j++){
        voltage = read_GPIO(as3310_kp->gpio_cols[j].port,
                            as3310_kp->gpio_cols[j].pin);
        if (voltage == 0) as3310_kp->ready2finish = 0;//some key is being pressed
    }
}

static void as3310_kp_timer_tingle(unsigned long data){

    update_state_new();
//    printstate();
    scan_state();
    kp_check_finish();
    if (as3310_kp->ready2finish == 1) as3310_kp->last_3times-- ; // all empty, back count 3 times.
    else as3310_kp->last_3times = 4;

    if (as3310_kp->last_3times == 0) {
        del_timer(&as3310_kp->timer_tingle);
        kp_io_irq_clr();
        kp_io_irq_unmask();
    }
    else  mod_timer(&as3310_kp->timer_tingle, jiffies + as3310_kp->delay); 

return;
}



void update_state_new(void){
//    printk("begin to update state\n");
    int i,j, voltage;

    as3310_kp->special_key.state_new = read_GPIO(as3310_kp->special_key.port,
                                                 as3310_kp->special_key.pin);

    for(i=0;i<ROW;i++){
        write_GPIO(as3310_kp->gpio_rows[i].port,
                   as3310_kp->gpio_rows[i].pin,0);  
    }

    for(j=0;j<COL;j++){
        voltage = read_GPIO(as3310_kp->gpio_cols[j].port,
                            as3310_kp->gpio_cols[j].pin);
        if (voltage==1) {//the col is hanging
            for(i=0;i<ROW;i++)
                as3310_kp->keypad_state_new[i][j]=0;
        }
        else{
//            for(i=0;i<ROW;i++) read_GPIO(as3310_kp->gpio_rows[i].port,as3310_kp->gpio_rows[i].pin);
            for(i=0;i<ROW;i++){
                write_GPIO(as3310_kp->gpio_rows[i].port,
                           as3310_kp->gpio_rows[i].pin,1);
                voltage = read_GPIO(as3310_kp->gpio_cols[j].port,
                                    as3310_kp->gpio_cols[j].pin);

                as3310_kp->keypad_state_new[i][j] = voltage;
                write_GPIO(as3310_kp->gpio_rows[i].port,
                           as3310_kp->gpio_rows[i].pin,0);
            }
        }
    }
}


void scan_state(void){
//    printk("begin to scane the state \n");
    int i,j,statesum,n,m;
    for(i=0;i<ROW;i++){
        for(j=0;j<COL;j++){
            if(as3310_kp->keypad_state[i][j]==as3310_kp->keypad_state_new[i][j]){
                if(as3310_kp->keypad_state_count[i][j]==1){
                    as3310_kp->keypad_state_count[i][j]=0;

                    if(as3310_kp->keypad_state_new[i][j]==0){//key up
                        if((*(as3310_kp->keymap + i*COL + j)==28)||
                           (*(as3310_kp->keymap + i*COL + j)==48)||
                           (*(as3310_kp->keymap + i*COL + j)==30)){
                                if(as3310_kp->functionkey_flag_start == 1) as3310_kp->functionkey_flag_start = 0;
                                else if(as3310_kp->functionkey_flag_F1 == 1) as3310_kp->functionkey_flag_F1 = 0;
                                else if(as3310_kp->functionkey_flag_F2 == 1) as3310_kp->functionkey_flag_F2 = 0;
                                else{
                                    #if keypaddubug
                                    printk("<<==key %d (Val = %d) is up ==>>\n",as3310_kp->key_map[i][j],
                                           *(as3310_kp->keymap + i*COL + j)  );
                                    #endif
                                    input_report_key(as3310_kp->input,*(as3310_kp->keymap + i*COL + j),0);
                                }
                        }
                        else{
                           #if keypaddubug
                   //        printstate();
                            printk("<<==key %d (Val = %d) is up ==>>\n",as3310_kp->key_map[i][j],
                                   *(as3310_kp->keymap + i*COL + j)  );
                           #endif
                            input_report_key(as3310_kp->input,*(as3310_kp->keymap + i*COL + j),0);
                        }

                    }
                    else{//key down
                        #if 0//CONFIG_AS3310x_MONITOR
                        combinekey();
                        #endif                        
                        statesum=0;
                        if((*(as3310_kp->keymap + i*COL + j)==28)||
                           (*(as3310_kp->keymap + i*COL + j)==48)||
                           (*(as3310_kp->keymap + i*COL + j)==30)){
                            for(n=0;n<ROW;n++){
                                for(m=0;m<COL;m++)
                                 statesum = statesum + as3310_kp->keypad_state_new[n][m];                                 
                            }
                            #if keypaddubug
                            printk(" the statesum is %d\n", statesum);
                            #endif
                            if(statesum>=3){
                                if(*(as3310_kp->keymap + i*COL + j)==28) as3310_kp->functionkey_flag_start = 1;
                                if(*(as3310_kp->keymap + i*COL + j)==30) as3310_kp->functionkey_flag_F1 = 1;
                                if(*(as3310_kp->keymap + i*COL + j)==48) as3310_kp->functionkey_flag_F2 = 1;
                            }
                            else {
                              #if keypaddubug
                              printk("<<==key %d (Val = %d) is down ==>>\n",as3310_kp->key_map[i][j],
                                       *(as3310_kp->keymap + i*COL + j));
                              #endif
                              input_report_key(as3310_kp->input,*(as3310_kp->keymap + i*COL + j),1);
                            }
                        }
                        else{
                              #if keypaddubug
                    //          printstate();
                              printk("<<==key %d (Val = %d) is down ==>>\n",as3310_kp->key_map[i][j],
                                       *(as3310_kp->keymap + i*COL + j));
                              #endif                       
                              input_report_key(as3310_kp->input,*(as3310_kp->keymap + i*COL + j),1);
                        }
                    }
                }
            }
            else{//state_new != state
                as3310_kp->keypad_state_count[i][j]=1;
            }
            as3310_kp->keypad_state[i][j] = as3310_kp->keypad_state_new[i][j];
        }
    }


    if(as3310_kp->special_key.state == as3310_kp->special_key.state_new){
        if(as3310_kp->special_key.state_count == 1){
            as3310_kp->special_key.state_count=0;
            if(as3310_kp->special_key.state_new == 0){//special key up
                #if keypaddubug
                printk("<<==key %d (Val = %d) is up ==>>\n",
                       as3310_kp->special_key.value,
                       as3310_kp->special_key.value);
                #endif
                input_report_key(as3310_kp->input,as3310_kp->special_key.value,0);
            }
            else{//key down
                #if keypaddubug
                printk("<<==key %d (Val = %d) is down ==>>\n",
                       as3310_kp->special_key.value,
                       as3310_kp->special_key.value);
                #endif
                input_report_key(as3310_kp->input,as3310_kp->special_key.value,1);            
            }
        }
    }
    else as3310_kp->special_key.state_count = 1;//state_new != state
    as3310_kp->special_key.state = as3310_kp->special_key.state_new;

}


void kp_io_irq_clr(void){
    int i;
    for(i=0;i<COL;i++)
        io_irq_clr(as3310_kp->gpio_cols[i].port,
                   as3310_kp->gpio_cols[i].pin);
    io_irq_clr(as3310_kp->special_key.port,
               as3310_kp->special_key.pin);
}

static void kp_io_irq_disable(void){
    int i;
    for(i=0;i<COL;i++)
        io_irq_disable(as3310_kp->gpio_cols[i].port,
                       as3310_kp->gpio_cols[i].pin);
}


static void kp_io_irq_mask(void){
int i;
    for(i=0;i<COL;i++)  
        io_irq_mask(as3310_kp->gpio_cols[i].port,
                    as3310_kp->gpio_cols[i].pin);
    io_irq_mask(as3310_kp->special_key.port,
                as3310_kp->special_key.pin);
}

void kp_io_irq_unmask(void){
int i;
     for(i=0;i<COL;i++)  
        io_irq_unmask(as3310_kp->gpio_cols[i].port,
                      as3310_kp->gpio_cols[i].pin);
     io_irq_unmask(as3310_kp->special_key.port,
                   as3310_kp->special_key.pin);
}



void init_as3310_kp_gpio_irq(void){

    int i;
    for(i=0;i<COL;i++)
        io_irq_enable_edge(as3310_kp->gpio_cols[i].port,
                           as3310_kp->gpio_cols[i].pin,
                           GPIO_IRQ_EDGE_FALLING);
    io_irq_enable_edge(as3310_kp->special_key.port,
                       as3310_kp->special_key.pin,
                       GPIO_IRQ_EDGE_RISING);
}

static irqreturn_t as3310_kp_interrupt(int irq, void *dev_id){

//    printk("======come into irq=========\n");
    as3310_kp->irq=irq;

    as3310_kp->last_3times = 4;

    kp_io_irq_mask();


    mod_timer(&as3310_kp->timer_tingle, jiffies + as3310_kp->delay);  

	return IRQ_HANDLED;
}

static void init_as3310_kp_state(void){
int i,j;
    for(j=0;j<COL;j++){
        for(i=0;i<ROW;i++){
            as3310_kp->keypad_state[i][j]=0;
            as3310_kp->keypad_state_count[i][j]=0;
            as3310_kp->keypad_state_new[i][j]=0;
        }
    }
    as3310_kp->irq_flag_old = COL;
    as3310_kp->irq_flag_new = COL;
    as3310_kp->irq_flag = 0;

    as3310_kp->special_key.state=0;
    as3310_kp->special_key.state_count=0;
    as3310_kp->special_key.state_new=0;
}



/*pin mux gpio and set output value=0*/
static int init_as3310_kp_gpio(void){

    int i;
    for(i=0;i<COL;i++){
        request_as3310_gpio(as3310_kp->gpio_cols[i].port,
                    as3310_kp->gpio_cols[i].pin,
                    PIN_FUNCTION_GPIO); 
    }
      
    for(i=0;i<ROW;i++){
        request_as3310_gpio(as3310_kp->gpio_rows[i].port,
                    as3310_kp->gpio_rows[i].pin,
                    PIN_FUNCTION_GPIO); 
    
        write_GPIO(as3310_kp->gpio_rows[i].port,
                   as3310_kp->gpio_rows[i].pin,0);  
    }

    request_as3310_gpio(as3310_kp->special_key.port,
                        as3310_kp->special_key.pin,
                        PIN_FUNCTION_GPIO);
}


static int as3310_kp_remove(struct platform_device *pdev){
	struct as3310_kp *as3310_kp = platform_get_drvdata(pdev);
	input_unregister_device(as3310_kp->input);
	kfree(as3310_kp);
	return 0;
}

/*kepad_state[]==1 indicates the key is being pressed*/
static void init_keymap(void){

    int i,j,map_value=0;
    for(i=0;i<ROW;i++){
        for(j=0;j<COL;j++){

            #if keypaddubug
            as3310_kp->key_map[i][j]=map_value;
            #endif
            as3310_kp->keypad_state[i][j]=0;

            map_value++;
        }
    }
}

static int __init as3310_kp_probe(struct platform_device *pdev)
{

//    dbg_printf("======come into probe=========\n");

    int ret,i,key;
    pdata = pdev->dev.platform_data;

    as3310_kp = kzalloc(sizeof(struct as3310_kp),GFP_KERNEL);
    as3310_kp->input = input_allocate_device(); 
    if (!as3310_kp || !as3310_kp->input) {
       kfree(as3310_kp);
       input_free_device(as3310_kp->input);
       return -ENOMEM;
    }

    platform_set_drvdata(pdev,as3310_kp);
    as3310_kp->input->evbit[0] = BIT(EV_KEY) | BIT(EV_REP);
    as3310_kp->keymap      =   pdata->keymap;
    as3310_kp->delay       =   pdata->delay;
    as3310_kp->special_key = pdata->special_key;

//    memcpy(as3310_kp->special_key,
//           pdata->special_key,
//           sizeof(pdata->special_key));

    memcpy(as3310_kp->gpio_rows,
           pdata->row_gpios,
           sizeof(pdata->row_gpios));

    memcpy(as3310_kp->gpio_cols,
           pdata->col_gpios,
           sizeof(pdata->col_gpios));

//    dbg_printf("======memcpy done=========\n");



	as3310_kp->input->name = "as3310-keypad";
    as3310_kp->input->phys = "as3310-keypad/input/event0";
    as3310_kp->input->dev.parent = &pdev->dev;
    //as3310_kp->input->private = as3310_kp;
    as3310_kp->input->id.bustype = BUS_HOST;
    as3310_kp->input->id.vendor = 0x0001;
    as3310_kp->input->id.product = 0x0001;
    as3310_kp->input->id.version = 0x0100;

    as3310_kp->input->keycode = as3310_kp->keymap;
    as3310_kp->input->keycodesize = sizeof(unsigned int);
    as3310_kp->input->keycodemax = pdata->keymapsize;

    init_keymap();
    for(key=0;key<(COL*ROW);key++){
         set_bit(*(as3310_kp->keymap+key) & KEY_MAX,as3310_kp->input->keybit);
     }

    setup_timer(&as3310_kp->timer_tingle, as3310_kp_timer_tingle, 0);

    ret = input_register_device(as3310_kp->input);
    if(ret < 0){ 
        goto err3;
 //       dbg_printf("======go to error 3=========\n");
    }

    init_as3310_kp_gpio();
    init_as3310_kp_gpio_irq();
     init_as3310_kp_state();

    for(i=0;i<4;i++){//register0 1,2,3 banks
        request_irq(INT_ASAP1820_PINCTRL0+i, as3310_kp_interrupt,
                   IRQF_SHARED, "as3310-keypad", as3310_kp);
    }

//    dbg_printf("======probe done=========\n");
  
return 0;

err3:
	device_remove_file(&pdev->dev, NULL);
//err1:
//	kfree(as3310_kp);KEYBOARD_AS3310_10
//	input_free_device(input_dev);

//err5:
//	for (i = irq_idx-1; i >=INT_AS3310_GPIO0; i--)
// 	free_irq(irq_idx, &pdev->dev);
//    return -EINVAL;
}




static struct platform_driver as3310_kp_driver = {
	.probe		= as3310_kp_probe,
	.remove		= as3310_kp_remove,
//	.suspend	= as3310_kp_suspend,
//	.resume		= as3310_kp_resume,
	.driver		= {
		.name	= "as3310-keypad",
 	},
};

static int __init as3310_kp_init(void)
{
	printk(KERN_INFO "AS3310 Keypad Driver\n");
	return platform_driver_register(&as3310_kp_driver);
}

static void __exit as3310_kp_exit(void)
{
	platform_driver_unregister(&as3310_kp_driver);
}

module_init(as3310_kp_init);
module_exit(as3310_kp_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("zhy");
MODULE_DESCRIPTION("as3310-Keypad Driver");


