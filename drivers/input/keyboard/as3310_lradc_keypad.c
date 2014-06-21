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
#include <mach/pincontrol.h>
#include <linux/mutex.h>
#include <linux/errno.h>
#include <mach/lradc_keypad.h>
#include <mach/lradc.h>
#include <asm/irq.h>
#include <mach/hardware.h>
#include <asm/io.h>
#include <asm/mach-types.h>
#include <mach/as3310x_monitor.h>
#include <mach/pwm.h>


#ifdef CONFIG_ASAP18XX_KEYPAD_DBG
    #define keypaddubug 1
#else
    #define keypaddubug 0
#endif

 

struct as3310_lradc_kp *as3310_lradc_kp;

struct as3310_lradc_kp_platform_data *pdata;

static int pskey=1;
static int exitkey=1;


static int as3310_lradc_kp_remove(struct platform_device *pdev){
	struct as3310_lradc_kp *as3310_lradc_kp = platform_get_drvdata(pdev);
	input_unregister_device(as3310_lradc_kp->input);
	kfree(as3310_lradc_kp);
	return 0;
}

void kp_lradc_scan(void)
{
    int ch,col;
    int key_serial;
    key_serial = 0;

    for(ch = 0;ch < CHANNEL_N;ch++){
       for(col = 0;col < as3310_lradc_kp->channel_colum_num[ch]; col++){
     
           if(LRADC_KEY_CMP_SHORT(as3310_lradc_kp->keypad_vol[ch], \
                                  as3310_lradc_kp->colum_vol[col])){


               if(as3310_lradc_kp->comp_times[ch] == 0)
                   {
                    as3310_lradc_kp->former_key[ch] = as3310_lradc_kp->keymap[key_serial];
                    as3310_lradc_kp->comp_times[ch]++;
               }
               else{
                   if(as3310_lradc_kp->former_key[ch] == as3310_lradc_kp->keymap[key_serial])
                   {
                        as3310_lradc_kp->comp_times[ch]++;
                        if(as3310_lradc_kp->comp_times[ch] == COMPARE_TIMES){
                            as3310_lradc_kp->comp_times[ch] = 0;
                            goto report_key;
                        }
                   }
                       
                   else if(as3310_lradc_kp->former_key[ch] != as3310_lradc_kp->keymap[key_serial])
                       as3310_lradc_kp->comp_times[ch] = 0;

               }
               goto next_for;


               report_key:

               if(as3310_lradc_kp->keypad_state[ch][col] == NON_TOUCHED){

                   input_report_key(as3310_lradc_kp->input,*(as3310_lradc_kp->keymap + key_serial),1);            


                   #if keypaddubug
                   printk("\t===============Key %d Pressed==============0x%x\n", \
                          *(as3310_lradc_kp->keymap + key_serial), as3310_lradc_kp->keypad_vol[ch] );
                   #endif
                   
                   as3310_lradc_kp->keypad_state[ch][col] = ALREAD_PRESSED;
               }
           }
           else{
               if(as3310_lradc_kp->keypad_state[ch][col] == ALREAD_PRESSED){
                   if(as3310_lradc_kp->comp_times[ch] >= COMPARE_TIMES) {
                       key_serial = col;
                       input_report_key(as3310_lradc_kp->input,*(as3310_lradc_kp->keymap + key_serial),0);            


                        #if keypaddubug
                            printk("\t===============Key %d Released==============0x%x\n", \
                                   *(as3310_lradc_kp->keymap + key_serial), as3310_lradc_kp->keypad_vol[ch]);
                        #endif
                        /* eliminate Doudong */
                        as3310_lradc_kp->keypad_vol[ch] = 0;
                        as3310_lradc_kp->keypad_state[ch][col] = NON_TOUCHED;
                        as3310_lradc_kp->comp_times[ch] = 0;
                   }
                   else{
                       as3310_lradc_kp->comp_times[ch]++;
                   }
                   
               }
           }
           next_for:
               key_serial++;
                       
        }
    }
   
}

void kp_lradc_timer_handler(void)
{   

    int ch, value;
    int ret;

    for(ch = 0; ch < CHANNEL_N; ch++){
        if(as3310_lradc_kp->channel_colum_num[ch]){
            as3310_lradc_kp->keypad_vol[ch] = lradc_get_value(as3310_lradc_kp->lradc_config->sample_num, ch);
            /*#if keypaddubug

            if(as3310_lradc_kp->keypad_vol[ch] != 0xfff)
             printk("ch %d is 0x%x\n",ch, as3310_lradc_kp->keypad_vol[ch]);
            #endif*/
        }
    }
  
    kp_lradc_scan();
#if 0
    value = read_GPIO(3,14);
    
    if(exitkey!=value)
    {
        exitkey=value;
   
            #ifdef CONFIG_AS3008
            input_report_key(as3310_lradc_kp->input,KEY_RIGHT,!exitkey);
            
            #endif

            #ifdef CONFIG_AS2808
            input_report_key(as3310_lradc_kp->input,KEY_EXIT,!exitkey);
            #endif
            
            #if keypaddubug
                printk("gpio(3, 14) value =%d",value);
            #endif
    }
   
    value=as3310_readl(0x8005c0b0)&0x8;
  
    if(pskey!=value)
    {
        pskey=value;
        #ifdef CONFIG_AS3008
        input_report_key(as3310_lradc_kp->input,KEY_EXIT,!value);
        #endif
        
        #ifdef CONFIG_AS2808
        input_report_key(as3310_lradc_kp->input,KEY_A,!value);
        #endif
        
        #if keypaddubug

        printk("gpio 2nd value = %d\n",!value);
        #endif
    }
#endif  


    ret = request_lradc_ch(as3310_lradc_kp->lradc_config, LRADC_NOT_USE_IRQ);

    if(ret != LRADC_OK)
       printk("Lradc channel request panic number: %d\n", ret);


    
    ret = lradc_set_timer(&as3310_lradc_kp->kp_lradc_timer, as3310_lradc_kp->delay, (void *)kp_lradc_timer_handler);
    if(ret != LRADC_OK)
       printk("Lradc timer set panic number: %d\n", ret);

    LAUNCH_LRADC_CHANNEL(as3310_lradc_kp->lradc_config->delay_reg_num);   
}   


static int kp_lradc_init(void){
   int ret; 
   as3310_lradc_kp->lradc_config->delay_reg_num = request_delay_reg_num();
 //  printk("keypad delay reg is %d\n", as3310_lradc_kp->lradc_config->delay_reg_num);
   if(as3310_lradc_kp->lradc_config->delay_reg_num == -1)
       printk("Lradc Delay regs have been run out\n");
   ret = request_lradc_ch(as3310_lradc_kp->lradc_config, LRADC_NOT_USE_IRQ);
   if(ret != LRADC_OK)
       printk("Lradc channel request panic number: %d\n", ret);
    
   ret = lradc_set_timer(&as3310_lradc_kp->kp_lradc_timer, as3310_lradc_kp->delay, (void *)kp_lradc_timer_handler);
   if(ret != LRADC_OK)
      printk("Lradc timer set panic number: %d\n", ret);

   LAUNCH_LRADC_CHANNEL(as3310_lradc_kp->lradc_config->delay_reg_num);   
   return 0;
}


static int __init as3310_lradc_kp_probe(struct platform_device *pdev)
{

    int ret, key;
   
    struct input_dev *input;
    pdata = pdev->dev.platform_data;

    as3310_lradc_kp = kzalloc(sizeof(struct as3310_lradc_kp),GFP_KERNEL);

   // request_as3310_gpio(3,14,3);
    input = input_allocate_device(); 
    if (!as3310_lradc_kp || !input) {
       kfree(as3310_lradc_kp);
       input_free_device(input);
       return -ENOMEM;
    }

    as3310_lradc_kp->input = input;

    platform_set_drvdata(pdev,as3310_lradc_kp);

    
    as3310_lradc_kp->keymap      =   pdata->keymap;
    as3310_lradc_kp->delay       =   pdata->delay;
    as3310_lradc_kp->lradc_config =  pdata->lradc_config;
    as3310_lradc_kp->channel_colum_num = pdata->channel_colum_num;
    as3310_lradc_kp->colum_vol = pdata->colum_vol;


    input->evbit[0] = BIT(EV_KEY) | BIT(EV_REP);
	input->name = "as3310-lradc-keypad";
    input->phys = "as3310-lradc-keypad/input/event0";
    input->dev.parent  = &pdev->dev;
    //input->private = as3310_lradc_kp;
    input->id.bustype = BUS_HOST;
    input->id.vendor = 0x0001;
    input->id.product = 0x0001;
    input->id.version = 0x0100;

    input->keycode = as3310_lradc_kp->keymap;
    input->keycodesize = sizeof(unsigned int);
    input->keycodemax = pdata->keymapsize;

    for(key = 0;key < pdata->keymapsize;key++)
         set_bit(*(as3310_lradc_kp->keymap + key) & KEY_MAX, input->keybit);
     


    ret = input_register_device(as3310_lradc_kp->input);
    if(ret < 0){ 
        goto err3;
    }

    

    kp_lradc_init(); 
    
    printk("Lradc Keypad Probe Over\n");
  
    return 0;

err3:
	device_remove_file(&pdev->dev, NULL);
    return -1;


}




static struct platform_driver as3310_lradc_kp_driver = {
	.probe		= as3310_lradc_kp_probe,
	.remove		= as3310_lradc_kp_remove,
	.driver		= {
		.name	= "as3310-lradc-keypad",
 	},
};

static int __init as3310_lradc_kp_init(void)
{
	printk(KERN_INFO "AS3310 Lradc Keypad Driver\n");
	return platform_driver_register(&as3310_lradc_kp_driver);
}

static void __exit as3310_lradc_kp_exit(void)
{
	platform_driver_unregister(&as3310_lradc_kp_driver);
}

module_init(as3310_lradc_kp_init);
module_exit(as3310_lradc_kp_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Ding Xiaopeng");
MODULE_DESCRIPTION("as3310-Lradc-Keypad Driver");


